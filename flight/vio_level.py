#!/usr/bin/env python3
"""Gravity-level cuVSLAM's odometry with PX4's attitude (shadow: publishes nothing to PX4).

cuVSLAM runs without IMU fusion, so its odom frame is the camera's pose at start-up: if the drone
was not level then (or the camera mount pitch is not modelled), the frame is tilted, and PX4 gets
vision positions whose "horizontal" is not horizontal. PX4's attitude is gravity-referenced.

While the drone is still, the world "up" direction is known in the body from PX4 (u_b), and in
cuVSLAM's odom frame it is u_o = R_odom_base @ u_b. The levelling rotation C is the smallest
rotation taking u_o onto +z; yaw is left alone (the bridge declares an FRD frame, arbitrary heading).
Output pose: R = C @ R_odom_base, p = C @ p_odom. Body-frame twist is unchanged.
C is re-estimated after a cuVSLAM tracking reset (vo_state back to 1).

Publishes /claude/vio_leveled/odometry (frame claude_odom_level -> base_link); status every 2 s
to the log and to vio_level_status.json.
"""
import argparse
import json
import math
import os
import time

import numpy as np

FLU_FRD = np.diag([1.0, -1.0, -1.0])
Z = np.array([0.0, 0.0, 1.0])


# ---------------------------------------------------------------- math (no ROS)
def quat_to_mat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


def up_in_body_flu(R_ned_frd):
    """World up expressed in the body (FLU), from PX4's attitude (FRD -> NED)."""
    return FLU_FRD @ (R_ned_frd.T @ np.array([0.0, 0.0, -1.0]))


def min_rotation(a, b=Z):
    """Smallest rotation taking unit vector a onto unit vector b."""
    a = a / np.linalg.norm(a)
    v = np.cross(a, b)
    s, c = np.linalg.norm(v), float(np.dot(a, b))
    if s < 1e-9:
        if c > 0:
            return np.eye(3)
        axis = np.cross(a, [1.0, 0, 0]) if abs(a[0]) < 0.9 else np.cross(a, [0, 1.0, 0])   # 180 deg
        axis /= np.linalg.norm(axis)
        return 2 * np.outer(axis, axis) - np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / (s * s))


def angle_deg(a, b):
    return math.degrees(math.acos(float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1))))


class Leveller:
    """Collects u_o while still, locks C; re-levels whenever still again and the residual exceeds
    relevel_deg (pure VO drifts in roll/pitch). Output transform p_l = C p_o + b, R_l = C R_ob; a
    re-level pivots about the current position, so the output position does not jump."""

    def __init__(self, n_init=30, max_spread_deg=1.5, still_rate=0.05, relevel_deg=0.5):
        self.n_init, self.max_spread, self.still_rate, self.relevel_deg = n_init, max_spread_deg, still_rate, relevel_deg
        self.C = None
        self.b = np.zeros(3)
        self.buf = []
        self.locks = 0
        self.relevels = 0
        self.lock_u = None
        self.last_correction_deg = 0.0

    def reset(self):
        self.C, self.b, self.buf = None, np.zeros(3), []

    def _still_mean(self):
        U = np.array(self.buf[-self.n_init:])
        m = U.mean(0)
        m /= np.linalg.norm(m)
        return m if max(angle_deg(u, m) for u in U) <= self.max_spread else None

    def add(self, R_odom_base, R_ned_frd, gyro_norm, p_odom=None):
        """Feed one paired sample. Returns the residual tilt (deg) once locked, else None."""
        u_o = R_odom_base @ up_in_body_flu(R_ned_frd)
        still = gyro_norm is not None and gyro_norm <= self.still_rate
        if not still:
            self.buf = []                                      # need a contiguous still stretch
        else:
            self.buf.append(u_o)
            self.buf = self.buf[-self.n_init:]
        if len(self.buf) >= self.n_init:
            m = self._still_mean()
            if m is None:
                self.buf = self.buf[-self.n_init // 2:]
            elif self.C is None:
                self.C, self.b, self.lock_u, self.locks = min_rotation(m), np.zeros(3), m, self.locks + 1
                self.buf = []
            elif angle_deg(self.C @ m, Z) > self.relevel_deg:
                C_new = min_rotation(m)
                if p_odom is not None:                         # pivot about the current position
                    p_now = self.C @ p_odom + self.b
                    self.b = p_now - C_new @ p_odom
                self.last_correction_deg = angle_deg(self.C @ m, Z)
                self.C, self.relevels = C_new, self.relevels + 1
                self.buf = []
        if self.C is None:
            return None
        return angle_deg(self.C @ u_o, Z)

    def apply(self, R_odom_base, p_odom):
        return self.C @ R_odom_base, self.C @ p_odom + self.b


def selftest():
    rng = np.random.default_rng(0)
    for trial in range(200):
        # true odom tilt (startup pose), random body attitudes
        C_true = quat_to_mat(*(lambda q: q / np.linalg.norm(q))(rng.normal(size=4)))     # odom <- level
        lev = Leveller(n_init=10)
        yaw = rng.uniform(-3, 3)
        roll, pitch = rng.uniform(-0.3, 0.3, 2)
        cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        R_ned_frd = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                     @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
        # level ENU/FLU attitude of the body (any yaw), then seen through the tilted odom frame
        T_enu_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
        R_level_base = T_enu_ned @ R_ned_frd @ FLU_FRD
        R_odom_base = C_true @ R_level_base
        for _ in range(12):
            lev.add(R_odom_base, R_ned_frd, 0.0)
        assert lev.C is not None
        R_l, _ = lev.apply(R_odom_base, np.zeros(3))
        tilt_l = math.degrees(math.acos(np.clip(R_l[2, 2], -1, 1)))
        tilt_true = math.degrees(math.acos(np.clip(R_level_base[2, 2], -1, 1)))
        assert abs(tilt_l - tilt_true) < 1e-6, (tilt_l, tilt_true)
        # a horizontal displacement in the level frame stays horizontal after levelling
        d_level = np.array([rng.normal(), rng.normal(), 0.0])
        d_odom = C_true @ d_level
        assert abs((lev.C @ d_odom)[2]) < 1e-9
    # re-level after roll/pitch drift: tilt fixed, position continuous
    lev = Leveller(n_init=10)
    R_level_base = np.eye(3)
    C0 = min_rotation(np.array([0.3, -0.2, 1.0]))
    p = np.array([2.0, -1.0, 0.5])
    for _ in range(12):                                        # PX4: level, yaw 0 (R_ned_frd = I)
        lev.add(C0.T @ R_level_base, np.eye(3), 0.0, C0.T @ p)
    drift = min_rotation(np.array([0.0, 0.35, 1.0]))          # VO picked up ~19 deg of tilt drift
    R_raw = drift @ C0.T @ R_level_base
    p_raw = drift @ C0.T @ p
    before = lev.apply(R_raw, p_raw)[1]
    for _ in range(25):                                        # a still stretch after the drift
        lev.add(R_raw, np.eye(3), 0.0, p_raw)
    R_l, after = lev.apply(R_raw, p_raw)
    assert lev.relevels == 1, lev.relevels
    assert math.degrees(math.acos(np.clip(R_l[2, 2], -1, 1))) < 1e-6
    assert np.allclose(before, after), (before, after)
    print("selftest ok: 200 random start-up tilts levelled exactly; horizontal stays horizontal; "
          "re-level after drift fixes tilt with no position jump")


# ---------------------------------------------------------------- ROS node
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--odom", default="/visual_slam/tracking/odometry")
    ap.add_argument("--out", default="/claude/vio_leveled/odometry")
    ap.add_argument("--usb-telemetry", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import SensorCombined, VehicleAttitude
    PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

    rclpy.init()
    n = Node("claude_vio_level")
    log = n.get_logger()
    pub = n.create_publisher(Odometry, a.out, 10)  # the organizers' bridge requires RELIABLE delivery
    lev = Leveller()
    st = {"att": None, "gyro": [], "vo": None, "vo_prev": None, "n_in": 0, "n_out": 0,
          "resid": [], "raw_tilt": None, "lev_tilt": None, "px4_tilt": None,
          "p_raw0": None, "p_lev0": None, "p_raw": None, "p_lev": None}
    status_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vio_level_status.json")

    def on_att(m):
        st["att"] = quat_to_mat(float(m.q[1]), float(m.q[2]), float(m.q[3]), float(m.q[0]))

    def on_imu(m):
        st["gyro"].append((time.time(), float(np.linalg.norm(m.gyro_rad))))
        st["gyro"] = [g for g in st["gyro"] if g[0] > time.time() - 0.3]

    def on_vo(m):
        v = int(m.vo_state)
        if st["vo"] is not None and st["vo"] != 1 and v == 1:
            lev.reset()                                         # cuVSLAM re-initialised: new frame
            st["p_raw0"] = st["p_lev0"] = None
            log.warn("cuVSLAM tracking reset: re-levelling (keep the drone still)")
        st["vo"] = v

    def on_odom(m):
        st["n_in"] += 1
        if st["att"] is None:
            return
        q, p = m.pose.pose.orientation, m.pose.pose.position
        R_ob = quat_to_mat(q.x, q.y, q.z, q.w)
        p_o = np.array([p.x, p.y, p.z])
        g = max((x[1] for x in st["gyro"]), default=None)
        locked_before = lev.C is not None
        r = lev.add(R_ob, st["att"], g, p_o)
        st["raw_tilt"] = math.degrees(math.acos(np.clip(R_ob[2, 2], -1, 1)))
        st["px4_tilt"] = math.degrees(math.acos(np.clip(st["att"][2, 2], -1, 1)))
        if lev.C is None:
            return
        if not locked_before:
            log.info(f"levelled: odom frame was tilted {angle_deg(lev.lock_u, Z):.1f} deg (lock #{lev.locks})")
        st["resid"].append(r if r is not None else 0.0)
        R_l, p_l = lev.apply(R_ob, p_o)
        st["lev_tilt"] = math.degrees(math.acos(np.clip(R_l[2, 2], -1, 1)))
        if st["p_raw0"] is None:
            st["p_raw0"], st["p_lev0"] = p_o.copy(), p_l.copy()
        st["p_raw"], st["p_lev"] = p_o, p_l
        out = Odometry()
        out.header = m.header
        out.header.frame_id = "claude_odom_level"
        out.child_frame_id = m.child_frame_id
        ql = mat_to_quat(R_l)
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = (float(v) for v in p_l)
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = (float(v) for v in ql)
        J = np.zeros((6, 6))
        J[:3, :3] = lev.C
        J[3:, 3:] = lev.C
        out.pose.covariance = (J @ np.array(m.pose.covariance).reshape(6, 6) @ J.T).ravel().tolist()
        out.twist = m.twist                                      # body frame: unchanged
        pub.publish(out)
        st["n_out"] += 1

    px4_out = "/claude/usb/out/" if a.usb_telemetry else "/fmu/out/"
    n.create_subscription(VehicleAttitude, px4_out + "vehicle_attitude", on_att, PX4_QOS)
    n.create_subscription(SensorCombined, px4_out + "sensor_combined", on_imu, PX4_QOS)
    n.create_subscription(Odometry, a.odom, on_odom, qos_profile_sensor_data)
    try:
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        n.create_subscription(VisualSlamStatus, "/visual_slam/status", on_vo, qos_profile_sensor_data)
    except ImportError:
        log.warn("VisualSlamStatus not importable: no reset handling")

    t_last = time.time()

    def report():
        nonlocal t_last
        dt = time.time() - t_last
        t_last = time.time()
        d = lambda k: None if st[k] is None or st[k + "0"] is None else (st[k] - st[k + "0"]).round(3).tolist()
        s = {
            "t": time.strftime("%T"), "locked": lev.C is not None, "locks": lev.locks,
            "relevels": lev.relevels, "last_correction_deg": round(lev.last_correction_deg, 2),
            "odom_tilt_at_lock_deg": None if lev.lock_u is None else round(angle_deg(lev.lock_u, Z), 2),
            "tilt_px4": None if st["px4_tilt"] is None else round(st["px4_tilt"], 2),
            "tilt_raw": None if st["raw_tilt"] is None else round(st["raw_tilt"], 2),
            "tilt_levelled": None if st["lev_tilt"] is None else round(st["lev_tilt"], 2),
            "residual_deg_max": round(max(st["resid"]), 2) if st["resid"] else None,
            "moved_raw_m": d("p_raw"), "moved_levelled_m": d("p_lev"),
            "rate_in_hz": round(st["n_in"] / dt, 1), "rate_out_hz": round(st["n_out"] / dt, 1),
            "vo_state": st["vo"], "still": bool(st["gyro"]) and max(x[1] for x in st["gyro"]) <= lev.still_rate,
        }
        st["n_in"] = st["n_out"] = 0
        st["resid"] = []
        with open(status_path + ".tmp", "w") as f:
            json.dump(s, f)
        os.replace(status_path + ".tmp", status_path)
        log.info(json.dumps(s))

    n.create_timer(2.0, report)
    log.info(f"{a.odom} + PX4 attitude -> {a.out} (shadow; nothing is sent to PX4)")
    rclpy.spin(n)


if __name__ == "__main__":
    main()

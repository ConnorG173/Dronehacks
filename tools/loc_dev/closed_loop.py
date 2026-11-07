#!/usr/bin/env python3
"""Offline closed-loop test of the whole localization chain (no Gazebo, no drone).

Ground truth flies a planner path (DHPATH) through the course. A fake VIO integrates the true
motion with injected drift (yaw drift per metre, scale error), starting from an arbitrary odom
origin. Depth images are raycast from the course wall grid (walls extruded floor..wall top, the
nets transparent, random dropouts). Fake PX4 topics give attitude, landed, armed and dist_bottom.
The real loc_fuser (enable_correction) and wall_matcher (source:=depth) run as subprocesses.
Errors of /team/odom_map vs truth are compared with the anchor-only (milestone 1) estimate.

    python3 tools/loc_dev/closed_loop.py --config planner/config/course_v3.yaml \
        --path <planner out>/path.bin [--yaw-drift 1.0] [--scale 1.02] [--time-scale 0.7] [--dropout 2.0:1.0]

Run inside the dev container with the workspace installed (see build_test.sh --shell).
This is a test harness: the path, drift and noise are test inputs, not flight data.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src", "team_loc_matcher"))
from team_loc_matcher import wallgrid  # noqa: E402

import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus  # noqa: E402
from px4_msgs.msg import VehicleAttitude, VehicleLandDetected, VehicleLocalPosition, VehicleStatus  # noqa: E402

G = 9.81
R_BODY_CAM = np.array([[0.0, 0, 1], [-1, 0, 0], [0, -1, 0]])  # optical (x right, y down, z fwd) -> FLU
T_BODY_CAM = np.array([0.10, 0.0, 0.03])


# ------------------------------------------------------------------------------ geometry
def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rpy_to_R(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rotz(yaw) @ Ry @ Rx


def R_to_q(R):
    """Rotation matrix -> (x, y, z, w)."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def enu_flu_to_px4_q(R_enu_flu):
    """Inverse of team_loc's px4_q_to_enu_flu: FLU->ENU rotation -> PX4 q (w, x, y, z) FRD->NED."""
    ned_enu = np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, -1]])
    frd_flu = np.diag([1.0, -1, -1])
    R = ned_enu @ R_enu_flu @ frd_flu
    x, y, z, w = R_to_q(R)
    return [w, x, y, z]


# ------------------------------------------------------------------------------ truth
class Truth:
    """Pad hold, climb, then the planner path (NED from StartPos/heading), then a hover."""

    def __init__(self, path_file, start, heading, hover, t_pad=3.0, t_climb=2.5, time_scale=1.0):
        raw = open(path_file, "rb").read()
        assert raw[:8] == b"DHPATH\0\0", "not a DHPATH file"
        rows = np.frombuffer(raw, "<f8", offset=32).reshape(-1, 12)
        c, s = math.cos(heading), math.sin(heading)
        t = rows[:, 0] / time_scale
        x = start[0] + c * rows[:, 1] + s * rows[:, 2]
        y = start[1] + s * rows[:, 1] - c * rows[:, 2]
        z = start[2] - rows[:, 3]
        ax = (c * rows[:, 7] + s * rows[:, 8]) * time_scale ** 2
        ay = (s * rows[:, 7] - c * rows[:, 8]) * time_scale ** 2
        yaw = heading - rows[:, 10]
        self.start, self.heading, self.hover = np.asarray(start, float), heading, hover
        self.t_pad, self.t_climb = t_pad, t_climb
        self.t_path0 = t_pad + t_climb
        self.tp, self.P = t, np.stack([x, y, z], 1)
        self.A, self.yaw = np.stack([ax, ay], 1), np.unwrap(yaw)
        self.t_end = self.t_path0 + t[-1] + 2.0

    def pose(self, t):
        """(p (3,), R FLU->course, landed)."""
        if t < self.t_pad:
            return self.start.copy(), rotz(self.heading), True
        if t < self.t_path0:
            f = (t - self.t_pad) / self.t_climb
            f = 0.5 - 0.5 * math.cos(math.pi * f)
            p = self.start + np.array([0, 0, self.hover * f])
            return p, rotz(self.heading), f < 0.02
        tt = min(t - self.t_path0, self.tp[-1])
        p = np.array([np.interp(tt, self.tp, self.P[:, i]) for i in range(3)])
        a = np.array([np.interp(tt, self.tp, self.A[:, i]) for i in range(2)])
        yaw = float(np.interp(tt, self.tp, self.yaw))
        # tilt toward the acceleration (body x forward): pitch nose down to accelerate forward
        fwd = np.array([math.cos(yaw), math.sin(yaw)])
        left = np.array([-math.sin(yaw), math.cos(yaw)])
        pitch = math.atan2(float(a @ fwd), G)
        roll = -math.atan2(float(a @ left), G)
        return p, rpy_to_R(roll, pitch, yaw), False


# ------------------------------------------------------------------------------ depth render
class DepthRenderer:
    def __init__(self, grid, w=160, h=100, hfov_deg=77.0, wall_top=1.7, rmax=4.5, step=0.04,
                 fill=0.55, noise=0.01, seed=0):
        self.w, self.h = w, h
        self.f = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
        self.cx, self.cy = w / 2, h / 2
        self.wall = grid["wall"]  # nets are not walls: the camera sees through them
        self.res, self.o = float(grid["res_m"]), np.array([grid["origin_x"], grid["origin_y"]])
        self.top, self.rmax, self.step = wall_top, rmax, step
        self.fill, self.noise = fill, noise
        self.rng = np.random.default_rng(seed)
        u, v = np.meshgrid((np.arange(w) - self.cx) / self.f, (np.arange(h) - self.cy) / self.f)
        self.rays_cam = np.stack([u, v, np.ones_like(u)], -1).reshape(-1, 3).astype(np.float32)
        self.s = np.arange(self.step, rmax, self.step, dtype=np.float32)

    def render(self, p_base, R_base):
        R = R_base @ R_BODY_CAM
        o = p_base + R_base @ T_BODY_CAM
        d = self.rays_cam @ R.T.astype(np.float32)        # (N, 3), optical z component 1
        o = o.astype(np.float32)
        n = np.linalg.norm(d, axis=1)
        dn = d / n[:, None]
        pts = o[None, None, :] + dn[:, None, :] * self.s[None, :, None]   # (N, S, 3)
        ij = np.floor((pts[..., :2] - self.o) / self.res).astype(np.int64)
        ok = (ij[..., 0] >= 0) & (ij[..., 0] < self.wall.shape[0]) & (ij[..., 1] >= 0) & (ij[..., 1] < self.wall.shape[1])
        occ = np.zeros(ok.shape, bool)
        occ[ok] = self.wall[ij[..., 0][ok], ij[..., 1][ok]]
        z = pts[..., 2]
        hit = (occ & (z >= 0) & (z <= self.top)) | (z <= 0)
        first = np.where(hit.any(1), hit.argmax(1), -1)
        rng_m = np.where(first >= 0, self.s[np.clip(first, 0, None)], np.nan)
        depth = rng_m / n                                  # along the optical axis
        depth *= 1 + self.noise * self.rng.standard_normal(depth.shape)
        depth[self.rng.random(depth.shape) > self.fill] = np.nan
        return depth.reshape(self.h, self.w).astype(np.float32)


# ------------------------------------------------------------------------------ harness node
class Harness(Node):
    def __init__(self, a, truth, renderer):
        super().__init__("closed_loop")
        self.a, self.truth, self.rend = a, truth, renderer
        self.t0 = time.monotonic()
        sd = qos_profile_sensor_data
        self.pub_odom = self.create_publisher(Odometry, "/visual_slam/tracking/odometry", 10)
        self.pub_vo = self.create_publisher(VisualSlamStatus, "/visual_slam/status", 10)
        self.pub_att = self.create_publisher(VehicleAttitude, "/fmu/out/vehicle_attitude", sd)
        self.pub_land = self.create_publisher(VehicleLandDetected, "/fmu/out/vehicle_land_detected", sd)
        self.pub_vs = self.create_publisher(VehicleStatus, "/fmu/out/vehicle_status_v1", sd)
        self.pub_lpos = self.create_publisher(VehicleLocalPosition, "/fmu/out/vehicle_local_position", sd)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)
        self.pub_depth = self.create_publisher(Image, "/team/loc/depth", sd)
        self.pub_info = self.create_publisher(CameraInfo, "/team/loc/depth/camera_info", sd)
        self.static = StaticTransformBroadcaster(self)
        tr = TransformStamped()
        tr.header.frame_id, tr.child_frame_id = "base_link", "cam_optical"
        tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = T_BODY_CAM
        q = R_to_q(R_BODY_CAM)
        tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = q
        self.static.sendTransform(tr)
        self.create_subscription(Odometry, "/team/odom_map", self.on_out, 100)
        self.create_subscription(DiagnosticStatus, "/team/loc/status", self.on_status, 50)
        self.create_subscription(String, "/team/loc/matcher_status", self.on_mstat, 50)

        # VIO state: odom origin away from the pad, drift accumulated per metre.
        self.T_odom_course_R = rotz(0.4)          # odom axes rotated vs course
        self.odom_origin = np.array([1.0, -2.0, 0.0])
        self.p_prev = None
        self.vio_p = None
        self.vio_yaw_err = 0.0
        self.vio_R = None
        self.dist = 0.0
        self.gt = {}                               # stamp ns -> (p, yaw)
        self.raw_anchor = None                     # anchor-only course pose model (as the fuser's M1)
        self.rows = []                             # (t, err_out_h, err_raw_h, err_out_yaw, err_raw_yaw, dist)
        self.status = []
        self.mstat = []
        self.jump = None
        if a.vio_jump:
            s_, dx = (float(v) for v in a.vio_jump.split(":"))
            self.jump = [truth.t_path0 + s_, dx, False]
        self.drop = None
        if a.dropout:
            s, d = (float(v) for v in a.dropout.split(":"))
            self.drop = (truth.t_path0 + s, truth.t_path0 + s + d)
        self.create_timer(1 / 30, self.tick_vio)
        self.create_timer(1 / a.depth_hz, self.tick_depth)
        self.create_timer(0.05, self.tick_px4)

    def now_t(self):
        return time.monotonic() - self.t0

    def stamp(self):
        return self.get_clock().now().to_msg()

    def vio_pose(self, p, R):
        """True pose -> drifting VIO pose in odom."""
        if self.p_prev is None:
            self.vio_p = self.odom_origin.copy()
            self.p_prev = p.copy()
            self.R0 = R.copy()
        dp = p - self.p_prev
        ds = float(np.linalg.norm(dp))
        self.dist += ds
        self.vio_yaw_err += math.radians(self.a.yaw_drift) * ds
        dp_odom = self.T_odom_course_R @ rotz(self.vio_yaw_err) @ dp * np.array([self.a.scale, self.a.scale, 1.0])
        self.vio_p = self.vio_p + dp_odom
        if self.jump is not None and not self.jump[2] and time.monotonic() - self.t0 >= self.jump[0]:
            self.vio_p = self.vio_p + np.array([self.jump[1], 0.0, 0.0])   # odom teleports along odom x
            self.vio_yaw_err += math.radians(10.0)                         # and 10 deg
            self.jump[2] = True
        self.p_prev = p.copy()
        R_odom = self.T_odom_course_R @ rotz(self.vio_yaw_err) @ R
        return self.vio_p.copy(), R_odom

    def tick_vio(self):
        t = self.now_t()
        p, R, landed = self.truth.pose(t)
        vp, vR = self.vio_pose(p, R)
        st = self.stamp()
        lost = self.drop is not None and self.drop[0] <= t < self.drop[1]
        vo = VisualSlamStatus()
        vo.header.stamp = st
        vo.vo_state = 2 if lost else 1
        self.pub_vo.publish(vo)
        tf = TransformStamped()
        tf.header.stamp = st
        tf.header.frame_id, tf.child_frame_id = "map", "odom"
        tf.transform.rotation.w = 1.0
        self.pub_tf.publish(TFMessage(transforms=[tf]))
        if lost:
            return
        m = Odometry()
        m.header.stamp = st
        m.header.frame_id, m.child_frame_id = "odom", "base_link"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = (float(v) for v in vp)
        q = R_to_q(vR)
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        k = st.sec * 10**9 + st.nanosec
        self.gt[k] = (p, math.atan2(R[1, 0], R[0, 0]), vp, vR)
        if landed or self.raw_anchor is None:
            # the M1 anchor model: launch pose * odom^-1, refreshed while landed (oracle-free)
            Rl = rotz(self.truth.heading)
            pl = self.truth.start
            Ra = Rl @ vR.T
            self.raw_anchor = (Ra, pl - Ra @ vp)
        self.pub_odom.publish(m)

    def tick_px4(self):
        t = self.now_t()
        p, R, landed = self.truth.pose(t)
        armed = t > 1.0
        vs = VehicleStatus()
        vs.arming_state = VehicleStatus.ARMING_STATE_ARMED if armed else VehicleStatus.ARMING_STATE_DISARMED
        self.pub_vs.publish(vs)
        ld = VehicleLandDetected()
        ld.landed = bool(landed)
        self.pub_land.publish(ld)
        att = VehicleAttitude()
        # PX4's heading is its own (arbitrary vs course): rotate by a constant 1 rad
        att.q = [float(v) for v in enu_flu_to_px4_q(rotz(1.0) @ R)]
        self.pub_att.publish(att)
        lp = VehicleLocalPosition()
        lp.dist_bottom = float(p[2] - self.truth.start[2])
        lp.dist_bottom_valid = True
        self.pub_lpos.publish(lp)

    def tick_depth(self):
        t = self.now_t()
        if self.drop is not None and self.drop[0] <= t < self.drop[1]:
            return
        p, R, _ = self.truth.pose(t)
        st = self.stamp()
        d = self.rend.render(p, R)
        img = Image()
        img.header.stamp, img.header.frame_id = st, "cam_optical"
        img.height, img.width, img.encoding = d.shape[0], d.shape[1], "32FC1"
        img.step = d.shape[1] * 4
        img.data = d.tobytes()
        info = CameraInfo()
        info.header = img.header
        info.width, info.height = d.shape[1], d.shape[0]
        info.k = [self.rend.f, 0.0, self.rend.cx, 0.0, self.rend.f, self.rend.cy, 0.0, 0.0, 1.0]
        self.pub_info.publish(info)
        self.pub_depth.publish(img)

    def on_out(self, m):
        k = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if k not in self.gt:
            return
        p, yaw, vp, vR = self.gt[k]
        q = m.pose.pose.orientation
        oyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        e_out = math.hypot(m.pose.pose.position.x - p[0], m.pose.pose.position.y - p[1])
        Ra, ta = self.raw_anchor
        pr = Ra @ vp + ta
        Rr = Ra @ vR
        e_raw = math.hypot(pr[0] - p[0], pr[1] - p[1])
        ey_out = math.degrees(math.atan2(math.sin(oyaw - yaw), math.cos(oyaw - yaw)))
        ryaw = math.atan2(Rr[1, 0], Rr[0, 0])
        ey_raw = math.degrees(math.atan2(math.sin(ryaw - yaw), math.cos(ryaw - yaw)))
        sig = math.sqrt(max(m.pose.covariance[0], m.pose.covariance[7]))
        self.rows.append((self.now_t(), e_out, e_raw, ey_out, ey_raw, self.dist, sig,
                          m.pose.pose.position.z - p[2]))

    def on_status(self, s):
        lvl = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
        self.status.append((self.now_t(), lvl, s.message, {kv.key: kv.value for kv in s.values}))

    def on_mstat(self, s):
        try:
            self.mstat.append((self.now_t(), json.loads(s.data)))
        except ValueError:
            pass


def start(cmd):
    return subprocess.Popen(cmd, start_new_session=True)


def stop(p):
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=5)
    except Exception:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--path", required=True, help="planner DHPATH file (planned from --config)")
    ap.add_argument("--yaw-drift", type=float, default=1.0, help="VIO yaw drift, deg per metre flown")
    ap.add_argument("--scale", type=float, default=1.02, help="VIO horizontal scale error")
    ap.add_argument("--time-scale", type=float, default=0.7, help="path playback speed factor")
    ap.add_argument("--depth-hz", type=float, default=10.0)
    ap.add_argument("--dropout", default="", help="VIO+depth dropout 'start_s:duration_s' after path start")
    ap.add_argument("--vio-jump", default="", help="VIO teleport 'start_s:dx_m' after path start (cuVSLAM reset)")
    ap.add_argument("--no-correction", action="store_true")
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    cfg = wallgrid.read_course_config(a.config)
    start_pos = cfg["StartPos"][:3]
    heading = math.radians(cfg.get("StartHeadingDeg", [0])[0])
    hover = cfg.get("HoverHeight", [1.0])[0]
    grid = wallgrid.build_from_files(None, a.config)
    truth = Truth(a.path, start_pos, heading, hover, time_scale=a.time_scale)
    rend = DepthRenderer(grid)

    fuser = start(["ros2", "run", "team_loc", "loc_fuser", "--ros-args", "-p", f"course_config:={a.config}",
                   "-p", f"enable_correction:={'false' if a.no_correction else 'true'}",
                   "-p", "base_height_on_pad_m:=0.0", "--log-level", "warn"])
    matcher = start(["ros2", "run", "team_loc_matcher", "wall_matcher", "--ros-args",
                     "-p", f"course_config:={a.config}", "-p", "source:=depth", "-p", "rate_hz:=5.0",
                     "--log-level", "warn"])
    time.sleep(2.0)
    rclpy.init()
    n = Harness(a, truth, rend)
    try:
        while n.now_t() < truth.t_end:
            rclpy.spin_once(n, timeout_sec=0.005)
    finally:
        stop(matcher)
        stop(fuser)

    R = np.array(n.rows)
    air = R[:, 0] > truth.t_path0
    out, raw = R[air, 1], R[air, 2]
    last = n.status[-1][3] if n.status else {}
    ms = [m for _, m in n.mstat if "ms_match" in m]
    summary = {
        "scenario": {"yaw_drift_deg_per_m": a.yaw_drift, "scale": a.scale, "time_scale": a.time_scale,
                     "dropout": a.dropout, "vio_jump": a.vio_jump, "correction": not a.no_correction},
        "path_m": round(float(R[-1, 5]), 2),
        "corrected": {"rms": float(np.sqrt(np.mean(out ** 2))), "max": float(out.max()), "end": float(out[-1]),
                      "yaw_rms_deg": float(np.sqrt(np.mean(R[air, 3] ** 2))), "z_rms": float(np.sqrt(np.mean(R[air, 7] ** 2)))},
        "raw_anchor_only": {"rms": float(np.sqrt(np.mean(raw ** 2))), "max": float(raw.max()), "end": float(raw[-1]),
                            "yaw_rms_deg": float(np.sqrt(np.mean(R[air, 4] ** 2)))},
        "fixes": {k: last.get(k) for k in ("fixes_in", "fixes_accepted", "fixes_rejected", "fixes_stale", "reacquired")},
        "matcher": {"attempts": len(ms), "ok": sum(1 for m in ms if m.get("ok")),
                    "ms_match_p50": float(np.median([m["ms_match"] for m in ms])) if ms else None,
                    "reasons": sorted({m.get("reason") for _, m in n.mstat if not m.get("ok")})[:8]},
        "status_levels_air": {str(k): sum(1 for s in n.status if s[0] > truth.t_path0 and s[1] == k) for k in (0, 1, 2)},
        "honesty_bad_ok": float(np.mean([(e > 0.3) for e, t in zip(R[:, 1], R[:, 0]) if t > truth.t_path0
                                         and any(s[1] == 0 and abs(s[0] - t) < 0.06 for s in n.status)]) if len(R) else 0),
        "sigma_vs_err_frac_over_3sig": float(np.mean(out > 3 * R[air, 6])),
    }
    print(json.dumps(summary, indent=1))
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        np.savetxt(os.path.join(a.out, "errors.csv"), R, delimiter=",",
                   header="t,err_out_h,err_raw_h,err_out_yaw_deg,err_raw_yaw_deg,dist_m,sigma_xy,err_out_z", comments="")
        json.dump(summary, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

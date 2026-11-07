"""wall_matcher: depth -> levelled BEV wall scan (~1 s stacked) -> correlative match against the
course map -> /team/loc/fix. The fuser (team_loc/loc_fuser) gates and fuses the fixes.

    subscribe  depth source, either
                 stereo: {left,right}_image_topic (mono8) + {left,right}_info_topic   (SGBM on CPU)
                 depth:  depth_topic (32FC1 m or 16UC1 mm) + depth_info_topic        (any producer)
               TF base_link <- camera optical frames (frame ids from camera_info headers)
               prior_topic        /team/odom_map (course pose at the image stamp, interpolated)
               odom_topic         /visual_slam/tracking/odometry (relative motion for stacking)
               attitude_topic     /fmu/out/vehicle_attitude (levelling: gravity from PX4's IMU)
               local_pos_topic    /fmu/out/vehicle_local_position (dist_bottom: floor-referenced band)
    publish    fix_topic          /team/loc/fix  PoseWithCovarianceStamped, frame course, image stamp
               status_topic       /team/loc/matcher_status  std_msgs/String JSON (one per attempt)

The wall grid is built at startup from map_file + course_config (team_loc_matcher.wallgrid).
Nothing is written to disk. Publishes nothing to PX4 or to setpoints.
"""
from __future__ import annotations

import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from team_loc_matcher import bev, wallgrid
from team_loc_matcher.match import CourseMap, MatchParams, gate, match


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PoseBuffer:
    """(t, x, y, z, yaw) samples; linear interpolation (yaw unwrapped) at a stamp."""

    def __init__(self, n=400):
        self.d = deque(maxlen=n)

    def add(self, t, x, y, z, yaw):
        if self.d and t <= self.d[-1][0]:
            if t < self.d[-1][0] - 1.0:   # time went backwards (sim reset / bag loop)
                self.d.clear()
            else:
                return
        self.d.append((t, x, y, z, yaw))

    def at(self, t, max_gap=0.1):
        if len(self.d) < 1:
            return None
        if t > self.d[-1][0]:
            return self.d[-1][1:] if t - self.d[-1][0] < max_gap * 0.5 else None
        if t < self.d[0][0]:
            return None
        lo, hi = 0, len(self.d) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if self.d[m][0] <= t:
                lo = m
            else:
                hi = m
        a, b = self.d[lo], self.d[hi]
        if b[0] - a[0] > max_gap:
            return None
        f = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
        dyaw = math.atan2(math.sin(b[4] - a[4]), math.cos(b[4] - a[4]))
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * (b[3] - a[3]), a[4] + f * dyaw)


def depth_from_msg(m):
    if m.encoding == "32FC1":
        a = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width]
        z = a.astype(np.float32)
    elif m.encoding in ("16UC1", "mono16"):
        a = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
        z = a.astype(np.float32) * 1e-3
    else:
        raise ValueError(f"unsupported depth encoding {m.encoding}")
    z = z.copy()
    z[~(z > 0)] = np.nan
    return z


class WallMatcher(Node):
    def __init__(self):
        super().__init__("wall_matcher")
        P = self.declare_parameter
        P("map_file", "")                  # empty: the course config's MapFile
        P("course_config", "")
        P("rate_hz", 5.0)
        P("source", "stereo")              # stereo | depth
        P("left_image_topic", "/oak/left/image_mono")
        P("right_image_topic", "/oak/right/image_mono")
        P("left_info_topic", "/oak/left/camera_info")
        P("right_info_topic", "/oak/right/camera_info")
        P("depth_topic", "/team/loc/depth")
        P("depth_info_topic", "/team/loc/depth/camera_info")
        P("base_frame", "base_link")
        P("prior_topic", "/team/odom_map")
        P("odom_topic", "/visual_slam/tracking/odometry")
        P("attitude_topic", "/fmu/out/vehicle_attitude")
        P("local_pos_topic", "/fmu/out/vehicle_local_position")
        P("fix_topic", "/team/loc/fix")
        P("status_topic", "/team/loc/matcher_status")
        P("scale", 0.5)                     # stereo: 1024x640 -> 512x320, as benchmarked on the drone
        P("num_disp", 96)
        P("block", 5)
        P("cv_threads", 2)
        P("stack_s", 1.0)
        P("rmax_m", 4.0)
        P("xy_window_m", 0.4)
        P("yaw_window_deg", 8.0)
        P("max_image_age_s", 0.5)          # skip a pair older than this (we fell behind)
        g = lambda n: self.get_parameter(n).value
        self.g = g

        if not g("course_config"):
            raise RuntimeError("course_config is required (map_file defaults to its MapFile)")
        t0 = time.perf_counter()
        grid = wallgrid.build_from_files(g("map_file"), g("course_config"))
        self.cmap = CourseMap.from_grid(grid)
        self.bp = bev.BevParams(rmax_m=float(g("rmax_m")), band_lo_agl_m=grid["band_lo_m"],
                                band_hi_agl_m=grid["band_hi_m"])
        self.mp = MatchParams(xy_window_m=float(g("xy_window_m")), yaw_window_deg=float(g("yaw_window_deg")))
        self.stack = bev.ScanStack(float(g("stack_s")), self.bp.res_m)
        self.get_logger().info(f"wall grid {grid['wall'].shape}: {int(grid['wall'].sum())} wall cells, "
                               f"{int(grid['boundary'].sum())} boundary cells "
                               f"({(time.perf_counter() - t0) * 1e3:.0f} ms)")

        self.prior = PoseBuffer()
        self.odom = PoseBuffer()
        self.att = None
        self.agl = None
        self.pair = None                   # (left, right) or (depth,)
        self.last_stamp = None
        self.stereo = None
        self.cam = None                    # (R_body_cam, t_body_cam, K)
        self.info = {}
        self.n_fix = self.n_try = 0

        self.fix_pub = self.create_publisher(PoseWithCovarianceStamped, g("fix_topic"), 10)
        self.status_pub = self.create_publisher(String, g("status_topic"), 10)
        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, g("prior_topic"), self.prior_cb, 50)
        self.create_subscription(Odometry, g("odom_topic"), self.odom_cb, sd)
        try:
            from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
            self.create_subscription(VehicleAttitude, g("attitude_topic"),
                                     lambda m: setattr(self, "att", tuple(m.q)), sd)
            self.create_subscription(VehicleLocalPosition, g("local_pos_topic"), self.lpos_cb, sd)
        except ImportError:
            self.get_logger().warn("px4_msgs not importable: levelling from the prior's attitude, body band")

        from tf2_ros import Buffer, TransformListener
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        if g("source") == "depth":
            self.create_subscription(CameraInfo, g("depth_info_topic"), lambda m: self.info.__setitem__("d", m), sd)
            self.create_subscription(Image, g("depth_topic"), lambda m: setattr(self, "pair", (m,)), sd)
        else:
            import message_filters
            self.create_subscription(CameraInfo, g("left_info_topic"), lambda m: self.info.__setitem__("l", m), sd)
            self.create_subscription(CameraInfo, g("right_info_topic"), lambda m: self.info.__setitem__("r", m), sd)
            subL = message_filters.Subscriber(self, Image, g("left_image_topic"), qos_profile=sd)
            subR = message_filters.Subscriber(self, Image, g("right_image_topic"), qos_profile=sd)
            self.sync = message_filters.ApproximateTimeSynchronizer([subL, subR], queue_size=4, slop=0.008)
            self.sync.registerCallback(lambda l, r: setattr(self, "pair", (l, r)))
        self.create_timer(1.0 / float(g("rate_hz")), self.tick)

    # ------------------------------------------------------------------ inputs
    def prior_cb(self, m):
        p = m.pose.pose.position
        self.prior.add(stamp_s(m.header), p.x, p.y, p.z, yaw_of_q(m.pose.pose.orientation))
        self.prior_att = m.pose.pose.orientation

    def odom_cb(self, m):
        p = m.pose.pose.position
        self.odom.add(stamp_s(m.header), p.x, p.y, p.z, yaw_of_q(m.pose.pose.orientation))

    def lpos_cb(self, m):
        if m.dist_bottom_valid and math.isfinite(m.dist_bottom):
            self.agl = (time.monotonic(), float(m.dist_bottom))

    def tf_lookup(self, target, source):
        try:
            tr = self.tf.lookup_transform(target, source, Time())
        except Exception:
            return None
        q, t = tr.transform.rotation, tr.transform.translation
        return bev.quat_to_mat(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def setup_camera(self):
        base = self.g("base_frame")
        if self.g("source") == "depth":
            d = self.info.get("d")
            if d is None:
                return False
            bc = self.tf_lookup(base, d.header.frame_id)
            if bc is None:
                return False
            self.cam = (bc[0], bc[1], (d.k[0], d.k[4], d.k[2], d.k[5]))
            self.get_logger().info(f"depth input ready ({d.width}x{d.height}, frame {d.header.frame_id})")
            return True
        L, R = self.info.get("l"), self.info.get("r")
        if L is None or R is None:
            return False
        lr = self.tf_lookup(L.header.frame_id, R.header.frame_id)
        bc = self.tf_lookup(base, L.header.frame_id)
        if lr is None or bc is None:
            return False
        import cv2
        from team_loc_matcher.depth import Stereo
        cv2.setNumThreads(int(self.g("cv_threads")))
        self.stereo = Stereo(L.k, L.d, R.k, R.d, (L.width, L.height), lr[0], lr[1],
                             float(self.g("scale")), int(self.g("num_disp")), int(self.g("block")))
        self.cam = (bc[0] @ self.stereo.R1.T, bc[1], self.stereo.K)
        self.get_logger().info(f"stereo ready {self.stereo.size}, f={self.stereo.f:.1f}, "
                               f"baseline={self.stereo.baseline:.4f} m")
        return True

    def level(self):
        if self.att is not None:
            return bev.level_rotation_from_px4(self.att)
        q = getattr(self, "prior_att", None)  # fallback (sim without PX4 topics): course attitude
        if q is None:
            return None
        return bev.level_rotation(bev.quat_to_mat(q.x, q.y, q.z, q.w))

    # ------------------------------------------------------------------ work
    def publish_status(self, st):
        self.status_pub.publish(String(data=json.dumps(st)))

    def tick(self):
        st = {"ok": False}
        if self.cam is None and not self.setup_camera():
            st["reason"] = "waiting for camera_info and TF"
            return self.publish_status(st)
        if self.pair is None:
            st["reason"] = "waiting for images"
            return self.publish_status(st)
        head = self.pair[0].header
        t = stamp_s(head)
        if t == self.last_stamp:
            return
        self.last_stamp = t
        now = self.get_clock().now().nanoseconds * 1e-9
        st["image_age_s"] = round(now - t, 3)
        if now - t > float(self.g("max_image_age_s")):
            st["reason"] = "image too old (behind)"
            return self.publish_status(st)
        R_level = self.level()
        if R_level is None:
            st["reason"] = "waiting for attitude"
            return self.publish_status(st)
        self.n_try += 1
        t0 = time.perf_counter()
        if self.stereo is not None:
            from team_loc_matcher.depth import img_to_np
            _, depth = self.stereo.depth(img_to_np(self.pair[0]), img_to_np(self.pair[1]))
        else:
            depth = depth_from_msg(self.pair[0])
        t1 = time.perf_counter()
        agl = self.agl[1] if self.agl and time.monotonic() - self.agl[0] < 1.0 else None
        fb = bev.frame_bev(depth, self.cam[2], self.cam[0], self.cam[1], R_level, agl, self.bp)
        o = self.odom.at(t)
        if o is None:
            st["reason"] = "no odometry at the image stamp"
            return self.publish_status(st)
        if not self.stack.add(t, fb, (o[0], o[1]), o[3]):
            st["stack_reset"] = True
        W, F = self.stack.scan()
        t2 = time.perf_counter()
        st.update(n_wall_pts=len(W), n_free_pts=len(F), depth_fill=round(float(np.isfinite(depth).mean()), 3),
                  ms_depth=round((t1 - t0) * 1e3, 1), ms_bev=round((t2 - t1) * 1e3, 1),
                  agl=None if agl is None else round(agl, 3))
        pr = self.prior.at(t)
        if pr is None:
            st["reason"] = "no prior (/team/odom_map) at the image stamp"
            return self.publish_status(st)
        prior = (pr[0], pr[1], pr[3])
        res = match(self.cmap, W, F, prior, self.mp)
        ok, why = gate(res, self.mp)
        st.update(ok=ok, reason=why, score=round(res.score, 3), n_in_cage=res.n_wall,
                  second_rel=round(res.second_rel, 3), pose=[round(float(v), 3) for v in res.pose],
                  prior=[round(float(v), 3) for v in prior],
                  sd=[round(float(v), 3) for v in np.sqrt(np.diag(res.cov))],
                  ms_match=round((time.perf_counter() - t2) * 1e3, 1))
        if ok:
            self.n_fix += 1
            m = PoseWithCovarianceStamped()
            m.header.stamp = head.stamp
            m.header.frame_id = "course"
            m.pose.pose.position.x, m.pose.pose.position.y = float(res.pose[0]), float(res.pose[1])
            m.pose.pose.position.z = float(pr[2])  # not estimated (huge variance below)
            m.pose.pose.orientation.z = math.sin(res.pose[2] / 2)
            m.pose.pose.orientation.w = math.cos(res.pose[2] / 2)
            C = np.zeros((6, 6))
            C[np.ix_([0, 1, 5], [0, 1, 5])] = res.cov
            C[2, 2] = C[3, 3] = C[4, 4] = 1e6
            m.pose.covariance = [float(v) for v in C.ravel()]
            self.fix_pub.publish(m)
        st.update(n_fix=self.n_fix, n_try=self.n_try)
        self.publish_status(st)


def main(args=None):
    rclpy.init(args=args)
    node = WallMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

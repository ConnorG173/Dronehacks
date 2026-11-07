"""course_matcher: stereo depth -> BEV -> ~1 s stack -> CAD match -> /localizer/fix. ~5 Hz. PASSIVE.

    subscribe  /oak/{left,right}/image_mono, camera_info, TF (base_link, optical frames)
               /fmu/out/vehicle_attitude       roll/pitch for levelling (IMU, not cuVSLAM)
               /fmu/out/vehicle_local_position dist_bottom for the height band
               /visual_slam/tracking/odometry  relative motion for stacking
               /localizer/pose                 the prior (path_streamer's T o odom)
    publish    /localizer/fix                  PoseWithCovarianceStamped, course frame, image stamp
               /localizer/matcher_status       JSON (gate result, score, points, ms)

Publishes nothing anyone flies on yet: path_streamer does not consume fixes (brief step 8).
"""
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from localizer import bev
from localizer.depth import Stereo, img_to_np
from localizer.match import CourseMap, MatchParams, gate, match

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class CourseMatcher(Node):
    def __init__(self):
        super().__init__("course_matcher")
        P = self.declare_parameter
        P("grid_file", "")
        P("rate_hz", 5.0)
        P("scale", 0.5)                 # 1024x640 -> 512x320, as benchmarked
        P("num_disp", 96)
        P("block", 5)
        P("cv_threads", 2)
        P("stack_s", 1.0)
        g = lambda n: self.get_parameter(n).value
        import cv2
        cv2.setNumThreads(int(g("cv_threads")))
        self.cmap = CourseMap.load(g("grid_file"))
        self.mp = MatchParams()
        self.bp = bev.BevParams()
        self.stack = bev.ScanStack(float(g("stack_s")), self.bp.res_m)
        self.cfg = {k: g(k) for k in ("scale", "num_disp", "block")}
        self.period = 1.0 / float(g("rate_hz"))

        self.infoL = self.infoR = None
        self.pair = None
        self.att = None                 # latest q (w, x, y, z)
        self.agl = None                 # (t_wall, m)
        self.odom = deque(maxlen=200)   # (stamp, x, y, yaw)
        self.prior = None               # (stamp, x, y, yaw) course
        self.stereo = None
        self.cam = None                 # (R_body_rect, t_body_cam)

        self.fix_pub = self.create_publisher(PoseWithCovarianceStamped, "/localizer/fix", 10)
        self.status_pub = self.create_publisher(String, "/localizer/matcher_status", 10)
        self.create_subscription(CameraInfo, "/oak/left/camera_info", lambda m: setattr(self, "infoL", m),
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/oak/right/camera_info", lambda m: setattr(self, "infoR", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/localizer/pose", self.prior_cb, 10)
        from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
        from skybeta_flight_interface.topics import fmu_topic
        self.create_subscription(VehicleAttitude, fmu_topic("out", "vehicle_attitude", "VehicleAttitude"),
                                 lambda m: setattr(self, "att", tuple(m.q)), PX4_QOS)
        self.create_subscription(VehicleLocalPosition,
                                 fmu_topic("out", "vehicle_local_position", "VehicleLocalPosition"),
                                 self.lpos_cb, PX4_QOS)
        import message_filters
        # mono8: a third of image_raw's bytes (the bench's rate shortfall)
        subL = message_filters.Subscriber(self, Image, "/oak/left/image_mono", qos_profile=qos_profile_sensor_data)
        subR = message_filters.Subscriber(self, Image, "/oak/right/image_mono", qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([subL, subR], queue_size=4, slop=0.008)
        self.sync.registerCallback(lambda l, r: setattr(self, "pair", (l, r)))
        from tf2_ros import Buffer, TransformListener
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.last_stamp = None
        self.create_timer(self.period, self.tick)
        self.get_logger().info("course_matcher up (passive: /localizer/fix is not flown on yet)")

    def odom_cb(self, m):
        q = m.pose.pose.orientation
        R = bev.quat_to_mat(q.x, q.y, q.z, q.w)
        p = m.pose.pose.position
        self.odom.append((stamp_s(m.header), p.x, p.y, bev.yaw_of(R)))

    def prior_cb(self, m):
        q = m.pose.orientation
        self.prior = (stamp_s(m.header), m.pose.position.x, m.pose.position.y,
                      2 * math.atan2(q.z, q.w))

    def lpos_cb(self, m):
        if m.dist_bottom_valid and math.isfinite(m.dist_bottom):
            self.agl = (time.monotonic(), float(m.dist_bottom))

    def odom_at(self, t):
        # HALF-ASSED: nearest odom sample, no interpolation (30 Hz: <= 17 ms off)
        if not self.odom:
            return None
        o = min(self.odom, key=lambda e: abs(e[0] - t))
        return o if abs(o[0] - t) < 0.05 else None

    def tf_lookup(self, target, source):
        try:
            tr = self.tf.lookup_transform(target, source, Time())
        except Exception:
            return None
        q, t = tr.transform.rotation, tr.transform.translation
        return bev.quat_to_mat(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def setup_stereo(self):
        lr = self.tf_lookup("oak_left_camera_optical_frame", "oak_right_camera_optical_frame")
        bc = self.tf_lookup("base_link", "oak_left_camera_optical_frame")
        if lr is None or bc is None or self.infoL is None or self.infoR is None:
            return False
        L, R = self.infoL, self.infoR
        self.stereo = Stereo(L.k, L.d, R.k, R.d, (L.width, L.height), lr[0], lr[1],
                             float(self.cfg["scale"]), int(self.cfg["num_disp"]), int(self.cfg["block"]))
        self.cam = (bc[0] @ self.stereo.R1.T, bc[1])
        self.get_logger().info(f"stereo ready {self.stereo.size}, f={self.stereo.f:.1f}, "
                               f"baseline={self.stereo.baseline:.4f} m")
        return True

    def tick(self):
        st = {"ok": False}
        if self.stereo is None and not self.setup_stereo():
            st["reason"] = "waiting for camera_info and TF"
            return self.status_pub.publish(String(data=json.dumps(st)))
        if self.pair is None or self.att is None:
            st["reason"] = "waiting for images and vehicle_attitude"
            return self.status_pub.publish(String(data=json.dumps(st)))
        l, r = self.pair
        t = stamp_s(l.header)
        if t == self.last_stamp:
            return
        self.last_stamp = t
        t0 = time.perf_counter()
        _, depth = self.stereo.depth(img_to_np(l), img_to_np(r))
        t1 = time.perf_counter()
        # HALF-ASSED: latest attitude, not the one at the image stamp
        R_level = bev.level_rotation_from_px4(self.att)
        agl = self.agl[1] if self.agl and time.monotonic() - self.agl[0] < 1.0 else None
        fb = bev.frame_bev(depth, self.stereo.K, self.cam[0], self.cam[1], R_level, agl, self.bp)
        o = self.odom_at(t)
        if o is None:
            st["reason"] = "no odometry at the image stamp"
            return self.status_pub.publish(String(data=json.dumps(st)))
        self.stack.add(t, fb, (o[1], o[2]), o[3])
        W, F = self.stack.scan()
        t2 = time.perf_counter()
        st.update(n_wall_pts=len(W), n_free_pts=len(F), depth_fill=round(float(np.isfinite(depth).mean()), 3),
                  ms_stereo=round((t1 - t0) * 1e3, 1), ms_bev=round((t2 - t1) * 1e3, 1))
        if self.prior is None or abs(self.prior[0] - t) > 0.2:
            st["reason"] = "no prior (/localizer/pose) near the image stamp"
            return self.status_pub.publish(String(data=json.dumps(st)))
        res = match(self.cmap, W, F, self.prior[1:], self.mp)
        ok, why = gate(res, self.mp)
        # HALF-ASSED: no Mahalanobis gate against a filter (there is no filter yet), and no
        # vo_state gate. The prior is T0 o odom, so drift shows up as fix - prior offsets.
        st.update(ok=ok, reason=why, score=round(res.score, 3), n_in_cage=res.n_wall,
                  second_rel=round(res.second_rel, 3), pose=[round(float(v), 3) for v in res.pose],
                  prior=[round(float(v), 3) for v in self.prior[1:]],
                  sd=[round(float(v), 3) for v in np.sqrt(np.diag(res.cov))],
                  ms_match=round((time.perf_counter() - t2) * 1e3, 1))
        if ok:
            m = PoseWithCovarianceStamped()
            m.header = l.header
            m.header.frame_id = "course"
            m.pose.pose.position.x, m.pose.pose.position.y = float(res.pose[0]), float(res.pose[1])
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(res.pose[2] / 2), math.cos(res.pose[2] / 2)
            C = np.zeros((6, 6))
            C[np.ix_([0, 1, 5], [0, 1, 5])] = res.cov
            C[2, 2] = C[3, 3] = C[4, 4] = 1e6                     # z, roll, pitch: not estimated
            m.pose.covariance = [float(v) for v in C.ravel()]
            self.fix_pub.publish(m)
        self.status_pub.publish(String(data=json.dumps(st)))


def main(args=None):
    rclpy.init(args=args)
    node = CourseMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

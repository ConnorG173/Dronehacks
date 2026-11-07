"""path_streamer: stream the planner's path as course-corrected setpoints. 20 Hz.

    subscribe  /visual_slam/tracking/odometry   odom pose (what the setpoints are expressed in)
               /skybeta/state                   armed / landed / offboard_active / PX4 local pos
               /fmu/out/vehicle_attitude        IMU tilt and heading (pad check, yaw report)
               /visual_slam/status              vo_state (optional)
    publish    <setpoint_topic>                 ENU PoseStamped in odom. Default the PREVIEW topic
                                                /localizer/setpoint_preview; /skybeta/setpoint/pose
                                                only when launched with live:=true (bench step)
               /localizer/pose                  vehicle pose in the course frame
               /localizer/status                JSON

Never talks to /fmu/in/*, never arms, never changes mode: skybeta_flight_interface owns all
of that and clamps altitude. Like square_mission: waits for offboard_active (the pilot arms,
then switches to Position), and stops advancing if Offboard ends.

States: INIT (needs landed + disarmed, computes T0 on the pad) -> READY (holds the current
position) -> TAKEOFF (climb to the path start) -> FOLLOW (time-pausing carrot) -> HOLD.
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import String

from localizer import bev
from localizer import streamer as S

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
LIVE_TOPIC = "/skybeta/setpoint/pose"


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class PathStreamer(Node):
    def __init__(self):
        super().__init__("path_streamer")
        P = self.declare_parameter
        P("path_file", "")
        P("grid_file", "")                       # for the pad pose (start_xy / start_heading_rad)
        P("setpoint_topic", "/localizer/setpoint_preview")
        P("rate_hz", 20.0)
        P("max_path_speed_mps", 1.0)             # the path is played slower to stay under this
        P("max_tracking_err_m", 0.6)             # carrot pauses beyond this
        P("takeoff_radius_m", 0.25)
        P("takeoff_settle_s", 2.0)
        P("fence_margin_m", 0.3)
        P("jump_m", 0.5)                         # odom step that counts as a VO jump
        P("command_yaw", True)
        P("dry_run", False)                      # preview only: pretend Offboard + perfect tracking
        g = lambda n: self.get_parameter(n).value

        self.topic = g("setpoint_topic")
        self.live = self.topic == LIVE_TOPIC
        self.dry_run = bool(g("dry_run"))
        if self.dry_run and self.live:
            raise SystemExit("dry_run is only allowed on the preview topic")
        with np.load(g("grid_file")) as z:
            self.pad_xy = np.asarray(z["start_xy"], float)
            self.pad_yaw = float(z["start_heading_rad"])
            self.cage_half = float(z["cage_half_m"])
        self.path = S.path_to_course(S.load_path(g("path_file")), self.pad_xy, self.pad_yaw)
        self.scale = S.time_scale_for(self.path, float(g("max_path_speed_mps")))
        self.p = {k: float(g(k)) for k in ("max_tracking_err_m", "takeoff_radius_m", "takeoff_settle_s",
                                           "fence_margin_m", "jump_m")}
        self.command_yaw = bool(g("command_yaw"))
        self.dt = 1.0 / float(g("rate_hz"))

        self.sp_pub = self.create_publisher(PoseStamped, self.topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/localizer/pose", 10)
        self.status_pub = self.create_publisher(String, "/localizer/status", 10)
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(String, "/skybeta/state", self.state_cb, 10)
        try:
            from px4_msgs.msg import VehicleAttitude
            from skybeta_flight_interface.topics import fmu_topic
            self.create_subscription(VehicleAttitude, fmu_topic("out", "vehicle_attitude", "VehicleAttitude"),
                                     self.att_cb, PX4_QOS)
        except ImportError as e:
            self.get_logger().error(f"no px4_msgs / flight interface: {e}; pad init will never pass")
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            self.create_subscription(VisualSlamStatus, "/visual_slam/status", self.vo_cb, qos_profile_sensor_data)
        except ImportError:
            pass

        self.odom = None            # (t_wall, xyz, yaw, tilt_deg)
        self.att = None             # (t_wall, tilt_deg, yaw_ned)
        self.fi = None              # (t_wall, dict) /skybeta/state
        self.vo_state = None
        self.mode = "INIT"
        self.reason = "waiting for odometry, attitude and /skybeta/state"
        self.T = None
        self.carrot = None
        self.settle_t0 = None
        self.last_sp = None         # (xy_course, z, yaw_course)
        self.err = None
        self.blocked = False
        self.create_timer(self.dt, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.create_timer(1.0, self.check_other_publishers)
        self.get_logger().info(
            f"path {self.path.duration:.1f} s, peak {self.path.speed.max():.2f} m/s, played at x{self.scale:.2f}"
            f" -> {self.path.duration / self.scale:.1f} s; setpoints -> {self.topic}"
            + ("  ** LIVE: flight_interface forwards these to PX4 **" if self.live else " (preview)")
            + ("  [dry run]" if self.dry_run else ""))

    # ------------------------------------------------------------------ inputs
    def odom_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        R = bev.quat_to_mat(q.x, q.y, q.z, q.w)
        xyz = np.array([p.x, p.y, p.z], float)
        if not np.isfinite(xyz).all():
            return
        if self.odom is not None and np.linalg.norm(xyz[:2] - self.odom[1][:2]) > self.p["jump_m"]:
            # HALF-ASSED: a jump only freezes the stream (hold the last setpoint, pilot takes over);
            # the brief's "inflate T, re-acquire with N consistent fixes" is not implemented.
            self.fault(f"odometry jumped {np.linalg.norm(xyz[:2] - self.odom[1][:2]):.2f} m in one step")
        self.odom = (time.monotonic(), xyz, bev.yaw_of(R), bev.tilt_deg(R))
        if self.T is not None:
            xy, yaw = self.T.to_course(xyz[:2], self.odom[2])
            self.pose_pub.publish(self.pose_msg("course", xy, xyz[2], yaw, m.header.stamp))

    def att_cb(self, m):
        R = bev.px4_to_enu_flu(m.q)
        yaw_ned = math.atan2(2 * (m.q[0] * m.q[3] + m.q[1] * m.q[2]), 1 - 2 * (m.q[2] ** 2 + m.q[3] ** 2))
        self.att = (time.monotonic(), bev.tilt_deg(R), yaw_ned)

    def state_cb(self, m):
        try:
            self.fi = (time.monotonic(), json.loads(m.data))
        except ValueError:
            pass

    def vo_cb(self, m):
        v = int(m.vo_state)
        if self.vo_state == 1 and v != 1:
            self.fault(f"vo_state {self.vo_state} -> {v}")
        self.vo_state = v

    def fresh(self, item, max_age=0.5):
        return item is not None and time.monotonic() - item[0] < max_age

    def fault(self, why):
        if self.mode in ("TAKEOFF", "FOLLOW", "READY"):
            self.get_logger().error(f"{why} -> HOLD (stream frozen; pilot takes over)")
            self.mode, self.reason = "HOLD", why
        elif self.mode == "INIT":
            self.reason = why

    # ------------------------------------------------------------------ safety
    def check_other_publishers(self):
        """Live only: another node publishing setpoints (square_mission after a stack restart)
        would interleave with ours in flight_interface. Refuse to publish while one exists."""
        if not self.live:
            return
        others = [i for i in self.get_publishers_info_by_topic(self.topic)
                  if i.node_name != self.get_name()]
        was = self.blocked
        self.blocked = bool(others)
        if self.blocked and not was:
            self.get_logger().error(
                f"another publisher on {self.topic}: {[i.node_name for i in others]} - NOT publishing")

    # ------------------------------------------------------------------ outputs
    def pose_msg(self, frame, xy, z, yaw, stamp=None):
        m = PoseStamped()
        m.header.stamp = stamp or self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(xy[0]), float(xy[1]), float(z)
        if yaw is not None:
            q = yaw_quat(yaw)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        else:
            m.pose.orientation.w = 0.0          # all-zero quaternion: do not command yaw
        return m

    def send_course(self, xy_c, z, yaw_c):
        """Course-frame setpoint -> odom via T^-1 -> setpoint topic. z is height, not transformed."""
        xy_c = S.clamp_to_fence(xy_c, self.cage_half, self.p["fence_margin_m"])
        self.last_sp = (xy_c, z, yaw_c)
        xy_o, yaw_o = self.T.to_odom(xy_c, yaw_c)
        self.send_odom(xy_o, z, yaw_o if self.command_yaw else None)

    def send_odom(self, xy_o, z, yaw_o):
        if self.blocked:
            return
        # HALF-ASSED: z is sent as "height above the floor" and relies on flight_interface's
        # unanchored z_ned = -z_enu and PX4's local z being rangefinder height (~0 on the pad),
        # i.e. a stack restarted on the pad. No odom-origin or rangefinder-offset compensation.
        self.sp_pub.publish(self.pose_msg("odom", xy_o, z, yaw_o))

    # ------------------------------------------------------------------ the 20 Hz state machine
    def vehicle_course(self):
        """(x, y, z) in the course frame: xy from T o odom, z from PX4's local position (height)."""
        xy = self.T.to_course(self.odom[1][:2])
        z = self.odom[1][2]
        if self.fresh(self.fi, 1.0) and self.fi[1].get("local_enu"):
            z = float(self.fi[1]["local_enu"][2])
        return np.r_[xy, z]

    def tick(self):
        if not self.fresh(self.odom):
            if self.mode in ("TAKEOFF", "FOLLOW"):
                self.fault("odometry stale")
            if self.last_sp is not None and self.T is not None:
                self.send_course(*self.last_sp)
            return
        fi = self.fi[1] if self.fresh(self.fi, 1.0) else {}
        offboard = self.dry_run or bool(fi.get("offboard_active", False))

        if self.mode == "INIT":
            self.step_init(fi)
            return
        if self.mode in ("TAKEOFF", "FOLLOW") and not offboard:
            self.get_logger().warn("Offboard ended -> HOLD. HALF-ASSED: restart the node for a new run.")
            self.mode, self.reason = "HOLD", "offboard ended"

        if self.mode == "READY":
            if offboard:
                self.mode, self.reason, self.settle_t0 = "TAKEOFF", "climbing to the path start", None
                self.get_logger().info("Offboard active -> TAKEOFF to the path start")
            else:
                # like square_mission: command where we are, so Offboard starts with a hold
                self.last_sp = None
                self.send_odom(self.odom[1][:2], self.odom[1][2], None)
                return

        if self.mode == "TAKEOFF":
            xy, z, yaw = self.path.at(0.0)
            self.send_course(xy, z, yaw)
            veh = np.r_[xy, z] if self.dry_run else self.vehicle_course()
            self.err = float(np.linalg.norm(veh - np.r_[xy, z]))
            if self.err > self.p["takeoff_radius_m"]:
                self.settle_t0 = None
            elif self.settle_t0 is None:
                self.settle_t0 = time.monotonic()
            elif time.monotonic() - self.settle_t0 > self.p["takeoff_settle_s"]:
                self.carrot = S.Carrot(self.path, self.scale, self.p["max_tracking_err_m"])
                self.mode, self.reason = "FOLLOW", "following the path"
                self.get_logger().info("at the path start -> FOLLOW")
            return

        if self.mode == "FOLLOW":
            if self.dry_run:
                xy, z, _ = self.path.at(self.carrot.tau)
                veh = np.r_[xy, z]
            else:
                veh = self.vehicle_course()
                if not S.inside_fence(veh[:2], self.cage_half, self.p["fence_margin_m"]):
                    self.fault(f"vehicle outside the course fence at ({veh[0]:.2f}, {veh[1]:.2f})")
                    self.send_course(*self.last_sp)
                    return
            (xy, z, yaw), self.err = self.carrot.step(self.dt, veh)
            self.send_course(xy, z, yaw)
            if self.carrot.done:
                self.mode, self.reason = "HOLD", "path complete - holding the goal; pilot lands"
                self.get_logger().info(self.reason)
            return

        if self.mode == "HOLD" and self.last_sp is not None:
            self.send_course(*self.last_sp)

    def step_init(self, fi):
        if not (self.fresh(self.att) and fi):
            self.reason = "waiting for attitude and /skybeta/state"
            return
        if fi.get("armed") or not fi.get("landed", False):
            # HALF-ASSED: initialisation only on the pad (landed, disarmed); no in-air (re)init.
            self.reason = "pad init needs the vehicle landed and disarmed"
            return
        T, why = S.pad_init(self.odom[1][:2], self.odom[2], self.odom[3], self.att[1],
                            self.pad_xy, self.pad_yaw)
        self.reason = why
        if T is None:
            return
        # HALF-ASSED: T is fixed from here on. No course_matcher fixes are fused (brief step 8),
        # so cuVSLAM drift goes uncorrected. The wide pad registration cross-check is not done.
        self.T = T
        self.mode, self.reason = "READY", "T0 set on the pad; waiting for Offboard"
        self.get_logger().info(f"pad init OK: T course<-odom = ({T.x:.3f}, {T.y:.3f}, "
                               f"{math.degrees(T.yaw):.2f} deg). Waiting for Offboard.")

    def publish_status(self):
        st = {"mode": self.mode, "reason": self.reason, "live": self.live, "dry_run": self.dry_run,
              "topic": self.topic, "blocked_by_other_publisher": self.blocked,
              "T": None if self.T is None else [round(self.T.x, 3), round(self.T.y, 3),
                                                round(math.degrees(self.T.yaw), 2)],
              "path_s": round(self.path.duration, 2), "time_scale": round(self.scale, 3),
              "tau": None if self.carrot is None else round(self.carrot.tau, 2),
              "carrot_paused": None if self.carrot is None else self.carrot.paused,
              "err_m": None if self.err is None else round(self.err, 3), "vo_state": self.vo_state}
        if self.T is not None and self.fresh(self.odom):
            st["vehicle_course"] = [round(v, 3) for v in self.vehicle_course()]
        if self.last_sp is not None:
            xy, z, yaw = self.last_sp
            xy_o, yaw_o = self.T.to_odom(xy, yaw)
            st["setpoint_course"] = [round(float(xy[0]), 3), round(float(xy[1]), 3), round(z, 3)]
            # what flight_interface will send (unanchored): NED = (y_enu, x_enu, -z), yaw = pi/2 - yaw
            st["expected_trajectory_setpoint_ned"] = [round(float(xy_o[1]), 3), round(float(xy_o[0]), 3),
                                                      round(-z, 3)]
            st["expected_yaw_ned_deg"] = round(math.degrees(S.wrap(math.pi / 2 - yaw_o)), 1)
        if self.fresh(self.att):
            st["px4_yaw_ned_deg"] = round(math.degrees(self.att[2]), 1)
            st["px4_tilt_deg"] = round(self.att[1], 2)
        if self.fresh(self.odom):
            st["odom_tilt_deg"] = round(self.odom[3], 2)
        self.status_pub.publish(String(data=json.dumps(st)))


def main(args=None):
    rclpy.init(args=args)
    node = PathStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

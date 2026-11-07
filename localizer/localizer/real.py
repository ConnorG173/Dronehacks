"""Real-drone adapter (skybeta Jetson, ROS 2 Humble): ROS messages <-> core dataclasses. 20 Hz.

    subscribe  /visual_slam/tracking/odometry   cuVSLAM odom: local xy + yaw (setpoints are in it)
               /skybeta/state                   JSON: armed, landed, offboard_active, local_enu
               /fmu/out/vehicle_attitude        IMU tilt (pad check) and PX4 yaw (status)
               /visual_slam/status              vo_state (optional)
    publish    /localizer/setpoint_preview      the lead point, PoseStamped ENU in odom (DEFAULT)
               /skybeta/setpoint/pose           the same, only with live:=true (bench / flight)
               /localizer/pose                  vehicle pose in the course frame
               /localizer/status                JSON, 2 Hz

This node never arms, never changes mode and has no /fmu/in publisher: skybeta_flight_interface
owns all of that, converts ENU -> NED as (y, x, -z) with yaw_ned = pi/2 - yaw_enu, clamps the
altitude to 0.3-3.0 m and sends position only (velocity/acceleration NaN), which is why the core's
lead point q is what goes out. The pilot arms on RC and switches to Offboard.
All decisions are in localizer.core.follower; this file only converts and publishes.
"""
import json
import math

import numpy as np

from localizer.core import frames as F
from localizer.core.follower import Config, Follower, VehicleState
from localizer.core.path import load_path, path_to_course

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from std_msgs.msg import String
except ImportError:          # laptop: only the conversion functions below are importable/tested
    rclpy, Node = None, object

PREVIEW_TOPIC = "/localizer/setpoint_preview"
LIVE_TOPIC = "/skybeta/setpoint/pose"
ODOM_TOPIC = "/visual_slam/tracking/odometry"
STATE_TOPIC = "/skybeta/state"


def real_config(**params):
    """Real-drone Config: the pilot arms, nobody auto-lands, pad checks on, z is PX4 height."""
    cfg = Config(**params)
    cfg.self_arm, cfg.land_at_end, cfg.land_after_hold_s = False, False, None
    cfg.on_offboard_lost, cfg.z_from_pad, cfg.wait_timeout_s = "hold", False, None
    return cfg


# ----------------------------------------------------------------------------- conversions

def odom_to_pose(msg):
    """nav_msgs/Odometry -> (xyz (3,), yaw, tilt_deg) in cuVSLAM odom (ENU-labelled, x forward)."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    R = F.quat_to_mat(q.x, q.y, q.z, q.w)
    return np.array([p.x, p.y, p.z], float), F.yaw_of(R), F.tilt_deg(R)


def parse_state(text):
    """/skybeta/state JSON -> (armed, landed, offboard, local_enu_z or None), or None if unparsable."""
    try:
        d = json.loads(text)
    except ValueError:
        return None
    if not isinstance(d, dict):
        return None
    enu = d.get("local_enu")
    z = float(enu[2]) if isinstance(enu, (list, tuple)) and len(enu) >= 3 and enu[2] is not None else None
    return bool(d.get("armed", False)), bool(d.get("landed", False)), bool(d.get("offboard_active", False)), z


def attitude_to_tilt_yaw(q_wxyz):
    """px4_msgs/VehicleAttitude.q -> (tilt_deg, yaw_ned)."""
    w, x, y, z = (float(v) for v in q_wxyz)
    return F.tilt_deg(F.px4_to_enu_flu((w, x, y, z))), math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def local_position(odom_xyz, state_z):
    """Local ENU position for the core: odom xy (what setpoints are expressed in) and PX4 local
    height from /skybeta/state (cuVSLAM z is not height: 269 m was observed). No height -> NaN."""
    return np.array([odom_xyz[0], odom_xyz[1], np.nan if state_z is None else state_z], float)


def fill_pose(msg, xyz, yaw, frame_id, stamp):
    """Write a PoseStamped. yaw None -> all-zero quaternion = flight_interface does not command yaw."""
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in xyz[:3])
    o = msg.pose.orientation
    o.x, o.y, o.z, o.w = (0.0, 0.0, 0.0, 0.0) if yaw is None else F.yaw_quat_xyzw(yaw)
    return msg


def flight_interface_ned(xyz_enu, yaw_enu):
    """What flight_interface sends PX4 for an ENU setpoint (unanchored): NED and yaw_ned."""
    return F.enu_to_ned(xyz_enu), None if yaw_enu is None else F.yaw_enu_to_ned(yaw_enu)


# ----------------------------------------------------------------------------- node

class RealAdapter(Node):
    def __init__(self):
        super().__init__("real_follower")
        P = self.declare_parameter
        P("path_file", "")
        P("live", False)                     # False: preview topic only. True: /skybeta/setpoint/pose
        P("rate_hz", 20.0)
        P("pad_x", -2.5146)                  # planner/config/course_v3.yaml StartPos
        P("pad_y", 0.9144)
        P("pad_heading_deg", 0.0)            # StartHeadingDeg (ASSUMED 0)
        P("cage_half_m", 3.049)
        P("max_path_speed_mps", 1.0)
        P("mpc_xy_p", 0.95)
        P("lag_ok_m", 0.3)
        P("max_tracking_err_m", 0.6)
        P("clock_slew_per_s", 1.0)
        P("takeoff_radius_m", 0.25)
        P("takeoff_settle_s", 2.0)
        P("fence_margin_m", 0.3)             # setpoint clamp
        P("fence_fault_margin_m", 0.15)      # vehicle outside -> HOLD
        P("jump_m", 0.5)
        P("command_yaw", True)
        P("dry_run", False)                  # preview only: pretend Offboard + perfect tracking
        g = lambda n: self.get_parameter(n).value

        self.live = bool(g("live"))
        self.topic = LIVE_TOPIC if self.live else PREVIEW_TOPIC
        cfg = real_config(
            pad_xy=(float(g("pad_x")), float(g("pad_y"))), pad_yaw=math.radians(float(g("pad_heading_deg"))),
            cage_half_m=float(g("cage_half_m")), dry_run=bool(g("dry_run")), command_yaw=bool(g("command_yaw")),
            **{k: float(g(k)) for k in ("max_path_speed_mps", "mpc_xy_p", "lag_ok_m", "max_tracking_err_m",
                                        "clock_slew_per_s", "takeoff_radius_m", "takeoff_settle_s",
                                        "fence_margin_m", "fence_fault_margin_m", "jump_m")})
        if cfg.dry_run and self.live:
            raise SystemExit("dry_run is only allowed on the preview topic")
        path = path_to_course(load_path(g("path_file")), cfg.pad_xy, cfg.pad_yaw)
        self.follower = Follower(path, cfg)
        self.state = VehicleState()
        self.odom_xyz = None
        self.state_z = None
        self.vo_state = None
        self.att = None                      # (tilt_deg, yaw_ned)
        self.blocked = False
        self.last_mode = None
        self.sent = None                     # (xyz_enu, yaw_enu) last published

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sp_pub = self.create_publisher(PoseStamped, self.topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/localizer/pose", 10)
        self.status_pub = self.create_publisher(String, "/localizer/status", 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(String, STATE_TOPIC, self.state_cb, 10)
        try:
            from px4_msgs.msg import VehicleAttitude
            from skybeta_flight_interface.topics import fmu_topic
            self.create_subscription(VehicleAttitude, fmu_topic("out", "vehicle_attitude", "VehicleAttitude"),
                                     self.att_cb, px4_qos)
        except ImportError as e:
            self.get_logger().error(f"no px4_msgs / flight interface: {e}; pad init will never pass")
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            self.create_subscription(VisualSlamStatus, "/visual_slam/status", self.vo_cb, qos_profile_sensor_data)
        except ImportError:
            pass
        self.create_timer(1.0 / float(g("rate_hz")), self.tick)
        self.create_timer(0.5, self.publish_status)
        self.create_timer(1.0, self.check_other_publishers)
        f = self.follower
        self.get_logger().info(
            f"path {path.duration:.1f} s, peak {path.speed.max():.2f} m/s, played at x{f.scale:.2f}"
            f" -> {path.duration / f.scale:.1f} s; setpoints -> {self.topic}"
            + ("  ** LIVE: flight_interface forwards these to PX4 **" if self.live else " (preview)")
            + ("  [dry run]" if cfg.dry_run else ""))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs (stamped on receipt)
    def odom_cb(self, m):
        xyz, yaw, tilt = odom_to_pose(m)
        if not np.isfinite(xyz).all():
            return
        s = self.state
        s.pose_stamp, s.pos, s.yaw, s.local_tilt_deg = self.now(), local_position(xyz, self.state_z), yaw, tilt
        s.estimate_ok = self.vo_state in (None, 1)
        f = self.follower
        if f.T is not None and np.isfinite(s.pos).all():
            xy, yaw_c = f.T.to_course(s.pos[:2], yaw)
            self.pose_pub.publish(fill_pose(PoseStamped(), np.r_[xy, s.pos[2] - f.z0], yaw_c, "course",
                                            m.header.stamp))

    def state_cb(self, m):
        parsed = parse_state(m.data)
        if parsed is None:
            return
        s = self.state
        s.armed, s.landed, s.offboard, self.state_z = parsed
        s.status_stamp = s.landed_stamp = self.now()

    def att_cb(self, m):
        self.att = attitude_to_tilt_yaw(m.q)
        self.state.imu_stamp, self.state.imu_tilt_deg = self.now(), self.att[0]

    def vo_cb(self, m):
        self.vo_state = int(m.vo_state)
        self.state.estimate_ok = self.vo_state == 1

    # ------------------------------------------------------------------ safety (ROS graph only)
    def check_other_publishers(self):
        """Live only: another node publishing setpoints (square_mission after a stack restart)
        would interleave with ours in flight_interface. Refuse to publish while one exists."""
        if not self.live:
            return
        others = [i for i in self.get_publishers_info_by_topic(self.topic) if i.node_name != self.get_name()]
        was, self.blocked = self.blocked, bool(others)
        if self.blocked and not was:
            self.get_logger().error(f"another publisher on {self.topic}: {[i.node_name for i in others]} - NOT publishing")

    # ------------------------------------------------------------------ the 20 Hz tick
    def tick(self):
        cmd = self.follower.step(self.now(), self.state)
        if cmd.mode != self.last_mode:
            self.get_logger().info(f"{self.last_mode} -> {cmd.mode}: {cmd.reason}")
            self.last_mode = cmd.mode
        if cmd.request_offboard_arm or cmd.request_land:
            self.get_logger().error("core asked to arm/land; the real adapter never does (ignored)")
        if cmd.publish and not self.blocked:
            # position only: the lead point q stands in for the feedforward (flight_interface drops v, a)
            self.sent = (cmd.q, cmd.q_yaw)
            self.sp_pub.publish(fill_pose(PoseStamped(), cmd.q, cmd.q_yaw, "odom", self.get_clock().now().to_msg()))

    def publish_status(self):
        st = self.follower.status()
        st.update({"live": self.live, "topic": self.topic, "blocked_by_other_publisher": self.blocked,
                   "vo_state": self.vo_state})
        if self.sent is not None:
            ned, yaw_ned = flight_interface_ned(*self.sent)
            st["expected_trajectory_setpoint_ned"] = [round(float(v), 3) for v in ned]
            st["expected_yaw_ned_deg"] = None if yaw_ned is None else round(math.degrees(yaw_ned), 1)
        if self.att is not None:
            st["px4_tilt_deg"], st["px4_yaw_ned_deg"] = round(self.att[0], 2), round(math.degrees(self.att[1]), 1)
        if self.state.local_tilt_deg is not None:
            st["odom_tilt_deg"] = round(self.state.local_tilt_deg, 2)
        self.status_pub.publish(String(data=json.dumps(st)))


def main(args=None):
    rclpy.init(args=args)
    node = RealAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

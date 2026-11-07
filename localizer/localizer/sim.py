"""Simulator adapter (DroneHacks simulator, ROS 2 Jazzy, native px4_msgs): ROS <-> core. 20 Hz.

    subscribe  /fmu/out/vehicle_local_position_v1, vehicle_status_v1, vehicle_land_detected,
               estimator_status_flags (names from px4_msgs MESSAGE_VERSION, like takeoff_land.py)
    publish    /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint (p, v, a, yaw, yawspeed:
               full feedforward), /fmu/in/vehicle_command (Offboard, arm, NAV_LAND)

    cd localizer && python3 -m localizer.sim                          # passive: publishes NOTHING
    cd localizer && python3 -m localizer.sim --i-have-team-approval   # arms and flies (simulator only)

Without --i-have-team-approval the node creates no publisher at all and only logs what it would
do. With it, it refuses to start unless FLYX_LAB_ACTIVE=1 and FLYX_HITL=0 (the lab notebook's own
simulator-only guard) and use_sim_time is true, so it can never arm a real flight controller. Use Reset world first. All QoS is PX4's BEST_EFFORT + TRANSIENT_LOCAL
depth 1. Time is the node clock (use_sim_time comes from the deployment parameter file, exactly as
onboard/examples/takeoff_land.py does). All decisions are in localizer.core.follower.
"""
import math
import os
import sys
from pathlib import Path

from localizer.core import frames as F
from localizer.core.follower import Config, Follower, VehicleState
from localizer.core.path import load_path, path_to_course

try:
    import rclpy
    from px4_msgs.msg import (EstimatorStatusFlags, OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                              VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
except ImportError:          # laptop: only the conversion functions below are importable/tested
    rclpy, Node = None, object

APPROVAL_FLAG = "--i-have-team-approval"
DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "course_v3_path.bin")
YAW_ALIGN_MAX_AGE_S = 3.0


def sim_config(approved, **params):
    """Simulator Config: we arm (only with approval), land at the end and after a fault hold,
    yield if Offboard is lost, and take T and the floor height from PX4's local frame on the pad
    (PX4 local NED is north-aligned, not the pad frame, so the odom-at-origin checks are off)."""
    cfg = Config(**params)
    cfg.self_arm, cfg.land_at_end, cfg.land_after_hold_s = bool(approved), True, 10.0
    cfg.on_offboard_lost, cfg.z_from_pad, cfg.wait_timeout_s = "stop", True, 45.0
    cfg.max_pad_offset_m = cfg.max_pad_yaw_deg = cfg.max_tilt_diff_deg = None
    cfg.ready_commands_yaw = True        # like takeoff_land: the priming setpoint holds the heading
    return cfg


def px4_topic(name, message_type):
    version = int(getattr(message_type, "MESSAGE_VERSION", 0))
    return "/fmu/out/" + name + (f"_v{version}" if version else "")


# ----------------------------------------------------------------------------- conversions

def local_position_to_pose(msg):
    """VehicleLocalPosition (NED) -> (pos local ENU (3,), yaw local ENU)."""
    return F.ned_to_enu((msg.x, msg.y, msg.z)), F.yaw_ned_to_enu(float(msg.heading))


def yaw_aligned(flags_msg, flags_age_s, pos_msg):
    """EstimatorStatusFlags.cs_yaw_align, fresh by receipt AND by PX4 time (takeoff_land.py)."""
    if flags_msg is None or flags_age_s is None:
        return False
    age_us = int(pos_msg.timestamp) - int(flags_msg.timestamp)
    return bool(flags_msg.cs_yaw_align and 0.0 <= flags_age_s <= YAW_ALIGN_MAX_AGE_S
                and int(flags_msg.timestamp) > 0 and 0 <= age_us <= int(YAW_ALIGN_MAX_AGE_S * 1e6))


def estimate_ok(pos_msg, aligned):
    """takeoff_land.py position_ready: xy/z valid, heading usable, all finite."""
    return bool(pos_msg.xy_valid and pos_msg.z_valid and (pos_msg.heading_good_for_control or aligned)
                and all(math.isfinite(float(v)) for v in (pos_msg.x, pos_msg.y, pos_msg.z, pos_msg.heading)))


def status_flags(msg):
    """VehicleStatus -> (armed, offboard, auto_land)."""
    return (msg.arming_state == msg.ARMING_STATE_ARMED, msg.nav_state == msg.NAVIGATION_STATE_OFFBOARD,
            msg.nav_state == msg.NAVIGATION_STATE_AUTO_LAND)


def fill_offboard_mode(msg, stamp_us):
    msg.timestamp = stamp_us
    msg.position = True          # position control; finite velocity/acceleration are feedforward
    return msg


def fill_trajectory_setpoint(msg, cmd, stamp_us):
    """Command (local ENU) -> TrajectorySetpoint (NED): the full feedforward p, v, a, yaw, yawspeed.
    HALF-ASSED: finite velocity/acceleration feedforward has never run in the simulator (no lab
    example sets it); the position-only fallback is to send cmd.q, cmd.q_yaw and NaN v, a."""
    msg.timestamp = stamp_us
    msg.position = [float(v) for v in F.enu_to_ned(cmd.p)]
    msg.velocity = [float(v) for v in F.enu_to_ned(cmd.v)]
    msg.acceleration = [float(v) for v in F.enu_to_ned(cmd.a)]
    if cmd.yaw is None:
        msg.yaw, msg.yawspeed = math.nan, math.nan
    else:
        msg.yaw, msg.yawspeed = float(F.yaw_enu_to_ned(cmd.yaw)), -float(cmd.yaw_rate)
    return msg


def fill_vehicle_command(msg, command, stamp_us, p1=0.0, p2=0.0):
    msg.timestamp = stamp_us
    msg.command = command
    msg.param1, msg.param2 = float(p1), float(p2)
    msg.target_system = msg.target_component = msg.source_system = msg.source_component = 1
    msg.from_external = True
    return msg


def simulator_only(env):
    """The lab notebook's guard for the isolated DroneHacks simulator (learn/ notebook, cell 41).
    HALF-ASSED: the variables are only known from the notebook and onboard/services.py."""
    return env.get("FLYX_LAB_ACTIVE") == "1" and env.get("FLYX_HITL") == "0"


def split_approval(argv):
    """(approved, argv without the flag)."""
    return APPROVAL_FLAG in argv, [a for a in argv if a != APPROVAL_FLAG]


def ros_arguments(argv):
    """Deployment parameter file (sim time in the simulator), exactly as takeoff_land.py."""
    configured = os.environ.get("DRONE_PARAMS_FILE")
    parameters = Path(configured or "/etc/flyx/mission.yaml")
    if parameters.is_file():
        return ["--ros-args", "--params-file", str(parameters), *argv]
    if configured:
        raise FileNotFoundError("Configured ROS parameter file is missing: " + str(parameters))
    return list(argv)


# ----------------------------------------------------------------------------- node

class SimAdapter(Node):
    def __init__(self, approved):
        super().__init__("sim_follower")
        P = self.declare_parameter
        P("path_file", DEFAULT_PATH)
        P("rate_hz", 20.0)
        P("pad_x", -2.5146)                  # planner/config/course_v3.yaml StartPos
        P("pad_y", 0.9144)
        P("pad_heading_deg", 0.0)            # StartHeadingDeg (ASSUMED 0)
        P("max_path_speed_mps", 1.0)
        g = lambda n: self.get_parameter(n).value
        if approved and not g("use_sim_time"):
            raise SystemExit(f"{APPROVAL_FLAG} refused: use_sim_time is false, this is not the simulator")
        self.approved = approved
        cfg = sim_config(approved, pad_xy=(float(g("pad_x")), float(g("pad_y"))),
                         pad_yaw=math.radians(float(g("pad_heading_deg"))),
                         max_path_speed_mps=float(g("max_path_speed_mps")))
        path = path_to_course(load_path(g("path_file")), cfg.pad_xy, cfg.pad_yaw)
        self.follower = Follower(path, cfg)
        self.state = VehicleState()
        self.flags, self.flags_t = None, None
        self.last_mode = None
        self.last_log = -1e9

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        if approved:                         # no approval: no publisher exists, nothing can be sent
            self.ocm_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
            self.sp_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
            self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.create_subscription(VehicleLocalPosition, px4_topic("vehicle_local_position", VehicleLocalPosition),
                                 self.on_position, qos)
        self.create_subscription(VehicleStatus, px4_topic("vehicle_status", VehicleStatus), self.on_status, qos)
        self.create_subscription(VehicleLandDetected, px4_topic("vehicle_land_detected", VehicleLandDetected),
                                 self.on_landed, qos)
        self.create_subscription(EstimatorStatusFlags, px4_topic("estimator_status_flags", EstimatorStatusFlags),
                                 self.on_flags, qos)
        self.create_timer(1.0 / float(g("rate_hz")), self.tick)
        self.get_logger().info(
            f"path {path.duration:.1f} s, peak {path.speed.max():.2f} m/s, played at x{self.follower.scale:.2f}; "
            + ("APPROVED: will request Offboard + arm, fly, and land" if approved else
               f"PASSIVE (no {APPROVAL_FLAG}): no publishers, nothing is sent"))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    # ------------------------------------------------------------------ inputs (stamped on receipt)
    def on_position(self, m):
        now, s = self.now(), self.state
        s.pose_stamp = now
        s.pos, s.yaw = local_position_to_pose(m)
        s.estimate_ok = estimate_ok(m, yaw_aligned(self.flags, None if self.flags_t is None else now - self.flags_t, m))

    def on_status(self, m):
        s = self.state
        s.armed, s.offboard, s.auto_land = status_flags(m)
        s.status_stamp = self.now()

    def on_landed(self, m):
        self.state.landed, self.state.landed_stamp = bool(m.landed), self.now()

    def on_flags(self, m):
        self.flags, self.flags_t = m, self.now()

    # ------------------------------------------------------------------ the 20 Hz tick
    def tick(self):
        now = self.now()
        if now <= 0.0:                       # sim time before the first /clock
            return
        cmd = self.follower.step(now, self.state)
        if cmd.mode != self.last_mode:
            self.get_logger().info(f"{self.last_mode} -> {cmd.mode}: {cmd.reason}")
            self.last_mode = cmd.mode
        if now - self.last_log >= 2.0:
            self.last_log = now
            st = self.follower.status()
            self.get_logger().info(f"{st['mode']}: tau {st['tau']} err {st['err_m']} rate {st['clock_rate']}")
        if not self.approved:
            return
        t = self.stamp_us()
        if cmd.publish:
            self.ocm_pub.publish(fill_offboard_mode(OffboardControlMode(), t))
            self.sp_pub.publish(fill_trajectory_setpoint(TrajectorySetpoint(), cmd, t))
        if cmd.request_offboard_arm:
            self.cmd_pub.publish(fill_vehicle_command(VehicleCommand(), VehicleCommand.VEHICLE_CMD_DO_SET_MODE, t, 1.0, 6.0))
            self.cmd_pub.publish(fill_vehicle_command(VehicleCommand(), VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, t, 1.0))
            self.get_logger().info("Offboard and arm requested")
        if cmd.request_land:
            self.cmd_pub.publish(fill_vehicle_command(VehicleCommand(), VehicleCommand.VEHICLE_CMD_NAV_LAND, t))


def main(argv=None):
    approved, argv = split_approval(sys.argv[1:] if argv is None else argv)
    if approved and not simulator_only(os.environ):
        print(f"{APPROVAL_FLAG} refused: needs FLYX_LAB_ACTIVE=1 and FLYX_HITL=0 (the DroneHacks simulator). "
              "The simulator adapter must never arm a real flight controller.", file=sys.stderr)
        return 2
    rclpy.init(args=ros_arguments(argv))
    node = SimAdapter(approved)
    try:
        while rclpy.ok() and node.follower.mode not in ("DONE", "STOPPED"):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warning("Node interrupted; verify vehicle state in QGroundControl")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if node.follower.mode == "DONE" else 1


if __name__ == "__main__":
    raise SystemExit(main())

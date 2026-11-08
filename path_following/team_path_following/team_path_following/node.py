"""mission_tracker: plan once, then fly the plan with velocity setpoints.

In:   /team/odom_map (pose in `course`, localization), /visual_slam/tracking/odometry
      (VIO, fallback), TF course->odom (localization's course->map + cuVSLAM's map->odom),
      /team/loc/status, /skybeta/state,
      /fmu/out/vehicle_local_position (read only, low-latency velocity; optional)
Out:  /skybeta/setpoint/twist (the only setpoint; never a pose), /team/plan/path,
      /team/mission/state, /team/tracker/debug

Frames (docs/CONTRACTS.md): all control in the planner's `course` frame (ENU; called "map" in
the pure modules); the command is rotated into cuVSLAM's odom ENU with the latest course->odom
(TF, else the launch-pose fallback captured at go).
"""
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String

from . import frames
from .clearance import ClearanceMap
from .course import load_course, read_dhmap, write_reverse_config
from .mission import Inputs, Mission, MissionParams, IDLE
from .pathfile import Trajectory
from .planner_runner import plan_best
from .tracker import TrackerParams

try:
    import tf2_ros
except ImportError:  # pragma: no cover
    tf2_ros = None
try:
    from px4_msgs.msg import VehicleLocalPosition
except ImportError:  # pragma: no cover
    VehicleLocalPosition = None

STATES = ['idle', 'takeoff', 'course', 'landing', 'landed', 'done', 'abort', 'return']
DEBUG_FIELDS = ['t', 'state', 'tau', 'taudot', 'e_along', 'e_cross', 'e_z',
                'vcmd_x', 'vcmd_y', 'vcmd_z', 'v_x', 'v_y', 'v_z', 'p_x', 'p_y', 'p_z',
                'ref_x', 'ref_y', 'ref_z', 'yaw_rate_cmd', 'pose_src', 'speed_scale', 'clearance']


class _Pose:
    """Latest pose sample, timed by arrival (robust to sim/wall clock mismatch)."""
    __slots__ = ('p', 'yaw', 'v', 'rx')

    def __init__(self, p, yaw, v, rx):
        self.p, self.yaw, self.v, self.rx = p, yaw, v, rx


def _odom_to_sample(msg, rx):
    q = msg.pose.pose.orientation
    yaw = frames.yaw_from_quat(q.x, q.y, q.z, q.w)
    p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    # twist is in the child (body FLU) frame: rotate by the full orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    tw = msg.twist.twist.linear
    v = r @ np.array([tw.x, tw.y, tw.z])
    if not (np.all(np.isfinite(p)) and math.isfinite(yaw)):
        return None
    return _Pose(p, yaw, v if np.all(np.isfinite(v)) else None, rx)


class MissionTracker(Node):

    def __init__(self):
        super().__init__('mission_tracker')
        d = self.declare_parameter
        d('course_config', '')
        d('path_file', '')                 # skip planning and fly this DHPATH file
        d('planner_binary', '')
        d('planner_overrides', 'MaxVelMag=1.2')   # "Key=value;Key=value"
        d('planner_candidates', 3)          # keep the fastest of N validated plans
        d('planner_budget_s', 45.0)
        d('out_dir', '/tmp/team_path_following')
        d('time_scale', 1.0)               # >1 slows the loaded plan uniformly
        d('mission_preset', 'course')     # course | hover (take off, hold, land on the pad) |
                                           # first_leg (to the first checkpoint slowly, hold, land there)
        d('hover_hold_s', 10.0)            # hover preset: hold time at HoverHeight over the pad
        d('first_leg_speed_mps', 0.4)      # first_leg preset: planner MaxVelMag
        d('first_leg_hold_s', 5.0)         # first_leg preset: hold at the checkpoint before landing
        d('return_config', '')             # '' off | 'reverse' (course goal -> start) | a config path
        d('return_path_file', '')          # fly this DHPATH file as the return instead of planning
        d('return_z_ceiling_m', 1.6)       # map z ceiling on the return leg (rule 7.7 exception)
        d('control_rate_hz', 50.0)
        d('pose_topic', '/team/odom_map')  # map-frame odometry (contract)
        d('pose_topic_frame', 'course')    # 'course' | 'odom'
        d('course_frame', 'course')
        d('odom_frame', 'odom')
        d('base_height_on_pad_m', 0.0)     # base_link above the floor on the pad (fallback only)
        d('vio_topic', '/visual_slam/tracking/odometry')
        d('use_tf', True)                  # course->odom from TF when available
        d('use_px4_velocity', True)
        d('px4_local_position_topic', '/fmu/out/vehicle_local_position')
        d('latency_comp_s', 0.06)          # pose age assumed at arrival (VIO pipeline)
        d('vel_agree_mps', 0.8)            # EKF2 velocity used only within this of the pose's velocity
        d('pose_timeout_s', 0.25)
        d('max_speed_mps', 1.5)            # must match the flight interface's
        d('speed_margin_mps', 0.1)
        d('z_ceiling_m', 1.6)              # map z; the interface does not clamp twists
        d('clearance_slow_m', 0.0)         # 0 disables clearance slow-down
        d('clearance_stop_m', 0.0)
        for k, v in vars(TrackerParams()).items():
            if k not in ('v_max', 'z_ceiling'):
                d('tracker.' + k, v)
        for k, v in vars(MissionParams()).items():
            d('mission.' + k, v)

        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.gp = gp
        tp = TrackerParams(**{k: gp('tracker.' + k) for k in vars(TrackerParams())
                              if k not in ('v_max', 'z_ceiling')})
        tp.v_max = gp('max_speed_mps') - gp('speed_margin_mps')
        tp.z_ceiling = gp('z_ceiling_m')
        self.tracker_params = tp
        self.mission_params = MissionParams(**{k: gp('mission.' + k) for k in vars(MissionParams())})

        self.course = load_course(gp('course_config'))
        self.mission = None
        self.traj = None
        self.clearance = None
        self.plan_error = None
        self.state_name = 'idle'

        # inputs
        self.pose = None          # primary (map or odom frame per pose_topic_frame)
        self.vio = None
        self.px4_vel = None       # (v_odom ENU, rx)
        self.fi_state = {}
        self.loc_level = 'OK'
        self.t_map_odom = None    # frames.Pose2, latest known
        self.t_map_odom_src = 'none'
        self.go_captured = False

        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                durability=QoSDurabilityPolicy.VOLATILE,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Odometry, gp('pose_topic'), self.pose_cb, 10)
        if gp('vio_topic') and gp('vio_topic') != gp('pose_topic'):
            self.create_subscription(Odometry, gp('vio_topic'), self.vio_cb, 10)
        self.create_subscription(DiagnosticStatus, '/team/loc/status', self.loc_cb, 10)
        self.create_subscription(String, '/skybeta/state', self.fi_state_cb, 10)
        if gp('use_px4_velocity') and VehicleLocalPosition is not None:
            px4_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 history=QoSHistoryPolicy.KEEP_LAST, depth=5)
            self.create_subscription(VehicleLocalPosition, self._px4_topic(), self.px4_cb, px4_qos)
        del sensor_qos

        self.tf_buffer = None
        if gp('use_tf') and tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.path_pub = self.create_publisher(Path, '/team/plan/path', latched)
        self.state_pub = self.create_publisher(String, '/team/mission/state', 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, '/team/tracker/debug', 10)

        self.dt = 1.0 / gp('control_rate_hz')
        self.timer = self.create_timer(self.dt, self.tick)
        self.create_timer(0.5, self.publish_state)
        self._last_clamp_warn = 0.0
        threading.Thread(target=self.prepare, daemon=True).start()

    # ------------------------------------------------------------ setup
    def _px4_topic(self):
        try:
            from skybeta_flight_interface.topics import resolve_endpoints
            return resolve_endpoints()['vehicle_local_position']
        except Exception:
            return self.gp('px4_local_position_topic')

    def prepare(self):
        """Plan (or load), convert, build the clearance map. Runs off the executor."""
        try:
            preset = self.gp('mission_preset')
            if preset not in ('course', 'hover', 'first_leg'):
                raise ValueError(f'mission_preset must be course, hover or first_leg, not {preset!r}')
            path_file = self.gp('path_file')
            overrides = {}
            for kv in filter(None, (s.strip() for s in self.gp('planner_overrides').replace('@', ';').split(';'))):
                k, v = kv.split('=', 1)
                overrides[k.strip()] = v.strip()
            if preset == 'hover':
                traj = Trajectory.hold(self.course.hover_point, self.course.start_heading_rad,
                                       float(self.gp('hover_hold_s')))
                path_file = 'hover'
            else:
                config = self.course.path
                if preset == 'first_leg':
                    if not self.course.checkpoints:
                        raise ValueError('first_leg preset needs at least one Checkpoint in the course config')
                    c = self.course.checkpoints[0].center
                    os.makedirs(self.gp('out_dir'), exist_ok=True)
                    config = os.path.join(self.gp('out_dir'), 'first_leg.yaml')
                    self.course.write_derived(config, {'Goal': f'[{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]'},
                                              drop=('Checkpoint',))
                    overrides['MaxVelMag'] = str(float(self.gp('first_leg_speed_mps')))
                    path_file = ''
                if not path_file:
                    path_file, summary = plan_best(
                        self.gp('planner_binary'), config, self.gp('out_dir'), overrides,
                        candidates=int(self.gp('planner_candidates')), budget_s=float(self.gp('planner_budget_s')),
                        log=self.get_logger().info)
                    self.get_logger().info(summary)
                traj = Trajectory.load(path_file, self.course)
                if preset == 'first_leg':
                    traj = traj.with_hold(float(self.gp('first_leg_hold_s')))
            if self.gp('time_scale') != 1.0:
                traj = traj.time_scaled(float(self.gp('time_scale')))
            self.get_logger().info(f'mission preset: {preset}')
            if self.gp('clearance_slow_m') > 0.0:
                self.clearance = ClearanceMap(read_dhmap(self.course.map_file), self.course.map_bound)
            rtraj, rcourse = (self.prepare_return(overrides if not self.gp('path_file') else {})
                              if preset == 'course' else (None, None))
            rtp = None
            if rtraj is not None:
                import copy
                rtp = copy.copy(self.tracker_params)
                rtp.z_ceiling = float(self.gp('return_z_ceiling_m'))
            self.traj = traj
            self.mission = Mission(traj, pad_z=float(self.course.start_pos[2]),
                                   tracker_params=self.tracker_params, params=self.mission_params,
                                   return_traj=rtraj,
                                   return_pad_z=float(rcourse.start_pos[2]) if rcourse is not None else None,
                                   return_tracker_params=rtp)
            self.publish_path()
            self.get_logger().info(
                f'plan ready: {traj.duration:.2f} s, {traj.length:.2f} m, '
                f'max speed {np.max(np.linalg.norm(traj.v, axis=1)):.2f} m/s; streaming zero twists')
        except Exception as exc:  # noqa: BLE001
            self.plan_error = str(exc)
            self.get_logger().error(f'NO PLAN, publishing nothing (no autonomy on arming): {exc}')

    def prepare_return(self, overrides):
        """(trajectory, course) of the optional return leg, or (None, None). A failed return plan
        never blocks the course: the mission then ends after the first landing."""
        cfg = self.gp('return_config')
        if not cfg:
            return None, None
        try:
            out = os.path.join(self.gp('out_dir'), 'return')
            os.makedirs(out, exist_ok=True)
            if cfg == 'reverse':
                cfg = write_reverse_config(self.course, os.path.join(out, 'return_config.yaml'))
            rcourse = load_course(cfg)
            path_file = self.gp('return_path_file')
            if not path_file:
                path_file, _ = plan_best(self.gp('planner_binary'), cfg, out, overrides,
                                         candidates=int(self.gp('planner_candidates')),
                                         budget_s=float(self.gp('planner_budget_s')), log=self.get_logger().info)
            rtraj = Trajectory.load(path_file, rcourse)
            if self.gp('time_scale') != 1.0:
                rtraj = rtraj.time_scaled(float(self.gp('time_scale')))
            self.get_logger().info(f'return plan ready: {rtraj.duration:.2f} s ({cfg})')
            return rtraj, rcourse
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'no return plan (course still flies, mission ends at the first landing): {exc}')
            return None, None

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = self.gp('course_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in self.traj.decimated(10):
            ps = PoseStamped()
            ps.header.frame_id = self.gp('course_frame')
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(c) for c in self.traj.p[i])
            yaw = float(self.traj.yaw[i])
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # ------------------------------------------------------------ callbacks
    def now(self):
        return time.monotonic()

    def pose_cb(self, msg):
        want = self.gp('course_frame') if self.gp('pose_topic_frame') != 'odom' else self.gp('odom_frame')
        if msg.header.frame_id and msg.header.frame_id != want and not getattr(self, '_frame_warned', False):
            self._frame_warned = True
            self.get_logger().warn(f'{self.gp("pose_topic")} frame_id "{msg.header.frame_id}", expected "{want}"')
        s = _odom_to_sample(msg, self.now())
        if s is not None:
            self.pose = s

    def vio_cb(self, msg):
        s = _odom_to_sample(msg, self.now())
        if s is not None:
            self.vio = s

    def px4_cb(self, msg):
        if not msg.v_xy_valid:
            return
        v_ned = (msg.vx, msg.vy, msg.vz)
        if all(math.isfinite(c) for c in v_ned):
            self.px4_vel = (np.array([v_ned[1], v_ned[0], -v_ned[2]]), self.now())

    def loc_cb(self, msg):
        level = msg.level
        if isinstance(level, (bytes, bytearray)):
            level = int.from_bytes(level, 'little')
        self.loc_level = {0: 'OK', 1: 'WARN'}.get(int(level), 'ERROR')   # ERROR(2), STALE(3)
        if self.loc_level != getattr(self, '_loc_logged', 'OK'):
            self._loc_logged = self.loc_level
            self.get_logger().warn(f'/team/loc/status -> {self.loc_level} ({msg.name}: {msg.message})')

    def fi_state_cb(self, msg):
        try:
            self.fi_state = json.loads(msg.data)
        except ValueError:
            return
        if self.fi_state.get('clamped') and self.now() - self._last_clamp_warn > 2.0:
            self._last_clamp_warn = self.now()
            self.get_logger().warn(f'flight interface clamped our setpoint: {self.fi_state["clamped"]}')

    # ------------------------------------------------------------ frames
    def update_map_odom(self):
        if self.tf_buffer is not None:
            try:
                tf = self.tf_buffer.lookup_transform(self.gp('course_frame'), self.gp('odom_frame'),
                                                     rclpy.time.Time())
                tr, q = tf.transform.translation, tf.transform.rotation
                self.t_map_odom = frames.Pose2(tr.x, tr.y, tr.z, frames.yaw_from_quat(q.x, q.y, q.z, q.w))
                self.t_map_odom_src = 'tf'
                return
            except Exception:  # noqa: BLE001
                pass
        if self.t_map_odom_src == 'tf':
            return   # keep the last localization correction
        # fallback (contract): launch pose, re-captured from VIO until the go signal
        vio = self.vio if self.vio is not None else (self.pose if self.gp('pose_topic_frame') == 'odom' else None)
        if vio is not None and not self.go_captured:
            launch = frames.map_from_launch(self.course)
            launch.z += float(self.gp('base_height_on_pad_m'))
            self.t_map_odom = frames.map_odom_at_go(launch, frames.Pose2(*vio.p, vio.yaw))
            self.t_map_odom_src = 'launch'

    def current_pose(self, now):
        """(pos_map, vel_map, yaw_map, age, source) with latency compensation."""
        timeout = self.gp('pose_timeout_s')
        lat = self.gp('latency_comp_s')
        T = self.t_map_odom
        src, p, yaw, v, rx = None, None, None, None, None
        if self.pose is not None and now - self.pose.rx < timeout:
            s = self.pose
            if self.gp('pose_topic_frame') != 'odom':
                src, p, yaw, v, rx = 1, s.p, s.yaw, s.v, s.rx
            elif T is not None:
                src, p, yaw = 1, T.apply(s.p), s.yaw + T.yaw
                v, rx = (T.apply_vec(s.v) if s.v is not None else None), s.rx
        if src is None and self.vio is not None and T is not None and now - self.vio.rx < timeout:
            s = self.vio
            src, p, yaw = 2, T.apply(s.p), s.yaw + T.yaw
            v, rx = (T.apply_vec(s.v) if s.v is not None else None), s.rx
        if src is None:
            return None, None, None, 1e9, 0
        if self.px4_vel is not None and T is not None and now - self.px4_vel[1] < 0.1:
            v_px4 = T.apply_vec(self.px4_vel[0])
            # EKF2 velocity is lower latency, but it can glitch (seen in sim after a wall strike:
            # -15 m/s while stationary). Use it only while it agrees with the pose's own velocity.
            if v is None or np.linalg.norm(v_px4 - v) < self.gp('vel_agree_mps'):
                v = v_px4
        age = now - rx
        if v is not None:
            # bounded latency compensation: a bad velocity must not move the estimate far
            vmax = self.gp('max_speed_mps') + 0.5
            n = float(np.linalg.norm(v))
            v_pred = v * (vmax / n) if n > vmax else v
            p = p + v_pred * min(age + lat, 0.15)
        return p, v, float(frames.wrap_pi(yaw)), age, src

    # ------------------------------------------------------------ loop
    def tick(self):
        if self.mission is None:
            return          # no plan: publish nothing, so arming gives no autonomy
        now = self.now()
        self.update_map_odom()
        fs = self.fi_state
        offboard = bool(fs.get('offboard_active', False))
        if offboard and not self.go_captured:
            self.go_captured = True
            self.get_logger().info(f'go: course->odom {self.t_map_odom} ({self.t_map_odom_src})')
        pos, vel, yaw, age, src = self.current_pose(now)
        clear, cscale = float('nan'), 1.0
        if self.clearance is not None and pos is not None:
            clear = self.clearance.distance(pos)
            cscale = self.clearance.speed_scale(pos, self.gp('clearance_slow_m'), self.gp('clearance_stop_m'))
        out = self.mission.step(self.dt, Inputs(
            now=now, offboard=offboard, armed=bool(fs.get('armed', False)), landed=fs.get('landed'),
            pos=pos, vel=vel, yaw=yaw, pose_age=age, loc_level=self.loc_level, clearance_scale=cscale))
        if out.note:
            self.get_logger().warn(out.note, throttle_duration_sec=1.0)
        if out.state != self.state_name:
            self.get_logger().info(f'mission: {self.state_name} -> {out.state}')
            self.state_name = out.state
            self.publish_state()

        v_map = out.v if np.all(np.isfinite(out.v)) else np.zeros(3)
        if self.t_map_odom is not None:
            v_odom = self.t_map_odom.inverse().apply_vec(v_map)
        else:
            v_odom = np.zeros(3)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(c) for c in v_odom)
        msg.twist.angular.z = float(out.yaw_rate) if math.isfinite(out.yaw_rate) else 0.0
        self.twist_pub.publish(msg)

        dbg = out.debug
        ref = dbg.get('p_ref', [math.nan] * 3)
        nan3 = [math.nan] * 3
        row = [now, STATES.index(out.state), dbg.get('tau', math.nan), dbg.get('taudot', math.nan),
               dbg.get('e_along', math.nan), dbg.get('e_cross', math.nan), dbg.get('e_z', math.nan),
               *v_map, *(vel if vel is not None else nan3), *(pos if pos is not None else nan3),
               *ref, out.yaw_rate, src, cscale, clear]
        dm = Float64MultiArray()
        dm.layout.dim.append(MultiArrayDimension(label=','.join(DEBUG_FIELDS), size=len(row), stride=len(row)))
        dm.data = [float(x) for x in row]
        self.debug_pub.publish(dm)

    def publish_state(self):
        msg = String()
        msg.data = self.state_name if self.mission is not None else ('abort' if self.plan_error else 'idle')
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionTracker()
    # A dead node in flight ends the run (the interface holds, then the pilot lands), so an
    # executor error (seen once in sim: rclpy take_message RuntimeError) is logged, not fatal.
    errors = 0
    while rclpy.ok():
        try:
            rclpy.spin(node)
            break
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            break
        except Exception as exc:  # noqa: BLE001
            errors += 1
            node.get_logger().error(f'executor error #{errors}, continuing: {exc!r}')
    try:
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.shutdown()

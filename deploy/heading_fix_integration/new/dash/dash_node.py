#!/usr/bin/env python3
"""skybeta-12 control dashboard with heartbeat-limited setpoints.

Serves dash.html on the Jetson's Tailscale address (token in the URL). The page polls
/api/state every 100 ms; each poll from the controlling page is the heartbeat. Setpoints go out
at 10 Hz only while that heartbeat is fresh (dash_core.DashCore).

DRY-RUN by default: publishes to /claude/dash/setpoint_preview/{pose,twist}, which nothing reads.
--live publishes /skybeta/setpoint/{pose,twist} for the organizers' flight_interface, and then
refuses to engage while any other node publishes /skybeta/setpoint/pose (e.g. square_mission).
Arm sends a normal VehicleCommand request; KILL sends a force disarm (param2=21196) that stops the
motors even in the air. Both live mode only; KILL needs no control claim.
Never forces commands or changes modes or parameters; PX4 checks still apply.
"""
import argparse
import copy
import json
import math
import os
import re
import secrets
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from px4_msgs.msg import VehicleCommand, VehicleCommandAck

from dash_core import DashCore, DashParams, Fc, Pos
from course_follow import WAYPOINT_ACCEPTANCE_M
from depth_cache import DepthOverlayCache
from course_state import CourseState
from course_safety import observations, MAX_COURSE_SPEED_MPS
from course_replan import plan_remaining
from state_wire import compact_state
from path_heading import yaw_quaternion

HERE = os.path.dirname(os.path.abspath(__file__))
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
ARM_SOURCE_COMPONENT = 197


def velocity_controller_limit(raw):
    """Require the interface's current-position velocity timeout behavior."""
    if not isinstance(raw,dict) or raw.get('velocity_timeout_hold_current') is not True:
        return None, 'Velocity tracker is waiting for the updated flight interface'
    envelope=raw.get('envelope')
    limit=envelope.get('max_speed_mps') if isinstance(envelope,dict) else None
    if type(limit) not in (int,float) or not math.isfinite(limit) or limit<=0:
        return None, 'Velocity command limit is unavailable'
    return min(float(limit),1.), ''


class DashNode(Node):
    def __init__(self, a):
        super().__init__("claude_dash")
        self.a = a
        self.lock = threading.Lock()
        self.core = DashCore(DashParams(hb_timeout_s=a.hb_timeout, fence_r_m=a.fence,
                                       min_alt_m=a.min_alt, max_alt_m=a.max_alt,
                                       v_xy=a.horizontal_speed,course_v_xy=a.course_speed,
                                       obstacle_mode='shadow', heading_mode='hold_start'))
        self.pos = None
        self.range_height = None
        self.fc = None
        self.fc_raw = {}
        self.vo_state = None
        self.vo_t = None
        self.batt = None
        self.vehicle = None
        self.land = None
        self.depth_overlay = DepthOverlayCache()
        self.course = CourseState(os.path.join(HERE, "course.json"))
        self.course_stop = threading.Event()
        self.last_course_sample_us = None
        self.last_course_resets = None
        self.create_subscription(String, '/claude/depth/overlay',
                                 lambda m: self.depth_overlay.update(m.data), 1)
        self.arm_request = None
        self._ground_sync = None
        self._ground_sync_retry_at = 0.
        self.arm_pub = None
        # USB telemetry is published only from newly received MAVLink messages.
        # Keep this namespace separate from the unreliable UART DDS feed.
        px4 = "/claude/usb" if a.usb_telemetry else "/fmu"
        if a.live:
            self.arm_pub = self.create_publisher(VehicleCommand, px4 + "/in/vehicle_command",
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE, depth=10))
            self.create_subscription(VehicleCommandAck, px4 + "/out/vehicle_command_ack", self._arm_ack, PX4_QOS)
        self.hold_client = self.create_client(Trigger, "/skybeta/hold")
        base = "/skybeta/setpoint" if a.live else "/claude/dash/setpoint_preview"
        self.pose_topic = base + "/pose"
        self.pub_pose = self.create_publisher(PoseStamped, base + "/pose", 10)
        self.pub_twist = self.create_publisher(TwistStamped, base + "/twist", 10)
        from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleLandDetected
        self.create_subscription(VehicleLocalPosition, px4 + "/out/vehicle_local_position", self._lpos, PX4_QOS)
        self.create_subscription(VehicleStatus, px4 + "/out/vehicle_status_v1", self._vehicle, PX4_QOS)
        self.create_subscription(VehicleLandDetected, px4 + "/out/vehicle_land_detected", self._land, PX4_QOS)
        self.create_subscription(String, "/skybeta/state", self._state, 10)
        # Team gyro heading fix (imu-fusion): only matters when its marker file switches it into the PX4 chain.
        self.heading = None
        self.create_subscription(String, "/team_heading/status", self._heading, 10)
        self.pub_heading_cal = self.create_publisher(String, "/team_heading/calibrate", 10)
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            self.create_subscription(VisualSlamStatus, "/visual_slam/status", self._vo, qos_profile_sensor_data)
        except ImportError:
            pass
        try:
            from px4_msgs.msg import BatteryStatus
            self.create_subscription(BatteryStatus, px4 + "/out/battery_status", self._batt, PX4_QOS)
        except ImportError:
            pass
        self.create_timer(0.1, self._tick)
        self.n_out = 0
        self.course_thread = threading.Thread(target=self._course_loop, daemon=True)
        self.course_thread.start()

    # ---- inputs
    def _lpos(self, m):
        now = time.monotonic()
        p = Pos(time.monotonic(), float(m.y), float(m.x), -float(m.z), math.pi / 2 - float(m.heading),
                bool(m.xy_valid and m.z_valid and all(math.isfinite(v) for v in (m.x, m.y, m.z, m.heading))))
        if (getattr(m, 'v_xy_valid', False)
                and math.isfinite(m.vx) and math.isfinite(m.vy)):
            p.vx, p.vy = float(m.vy), float(m.vx)
        if getattr(m, 'v_z_valid', False) and math.isfinite(m.vz):
            p.vz = -float(m.vz)
        with self.lock:
            previous = self.pos
            self.pos = p
            self.range_height = (now, float(m.dist_bottom), bool(m.dist_bottom_valid
                and m.dist_bottom_sensor_bitfield & 1 and math.isfinite(m.dist_bottom)))
        sample = int(m.timestamp_sample)
        resets = tuple(getattr(m, key, None) for key in
                       ('xy_reset_counter','z_reset_counter','heading_reset_counter'))
        previous_resets = getattr(self, 'last_course_resets', None)
        self.last_course_resets = resets
        if (self.last_course_sample_us is not None and sample > 0
                and sample < self.last_course_sample_us - 1_000_000):
            self.course.invalidate_anchor("PX4 time restarted; set the course reference again")
            with self.lock:
                self.core.home = None
        elif previous is not None and now - previous.t > 2:
            self.course.invalidate_anchor("Position delivery was interrupted; set the course reference again")
            with self.lock:
                self.core.home = None
        elif (previous_resets is not None and all(v is not None for v in resets)
                and resets != previous_resets):
            self.course.invalidate_anchor('PX4 reset its position/heading reference; set home again')
            with self.lock:
                self.core.home = None
        if sample > 0:
            self.last_course_sample_us = sample

    def _course_loop(self):
        # Preview math never runs in the ROS timer or under the flight lock.
        try:
            os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), 10)
        except (OSError, AttributeError):
            pass
        while not self.course_stop.wait(.5):
            if self.core.p.obstacle_mode == 'shadow':
                continue  # Depth still feeds the display; no collision planner runs.
            try:
                if not DashNode.plan_paused_course(self) and not DashNode.plan_nudge(self):
                    self.course.update_preview(DashNode.depth_for_route(self))
            except Exception as exc:
                self.get_logger().error("Course preview unavailable: " + str(exc))
                self.course.update_preview({"status": "error", "reason": "Preview processing unavailable"})

    def depth_for_route(self, now=None, route=None):
        """Depth for planning and the guard: only cells at the route's height near them.
        The map display keeps the unfiltered snapshot."""
        from depth_height import filter_for_route
        depth = self.depth_overlay.snapshot() if now is None else self.depth_overlay.snapshot(now)
        if route is None:
            run = self.core.course_run
            if run is not None and getattr(run, 'active', False):
                route = run.points
            else:
                try:
                    route = self.course.world_route()
                except Exception:
                    route = None
        return filter_for_route(depth, route)

    def plan_nudge(self):
        """Plan an in-flight nudge outside the flight lock; apply happens in _tick."""
        now = time.monotonic()
        with self.lock:
            request = self.core.nudge_request
            run = self.core.course_run
            if request is None or request.get('path') is not None:
                return request is not None
            if run is None or request['run'] is not run or not run.active:
                self.core.nudge_request = None
                return False
            guard = run.obstacle_guard
            points, times, elapsed = list(run.points), list(run.times), run.elapsed
            start = tuple(self.core.cmd)
            walls = guard.known_walls()
            floor, ceiling = self.core._floor(), self.core._ceiling()
            depth = DashNode.depth_for_route(self, now, run.points)
        try:
            path = plan_remaining(points, times, elapsed, start, run.anchor, run.reference,
                run.radius, depth, walls, known_cell_m=guard.cell_m, clearance_m=guard.clearance,
                floor=floor, ceiling=ceiling)
            why = None if path.get('source') == 'adjusted' else 'route already clear'
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            path, why = None, str(exc)
        with self.lock:
            if self.core.nudge_request is request:
                if why is None:
                    request.update(path=path, planned_at=now)
                else:
                    # No nudge: keep flying the original route; retry shortly.
                    self.core.nudge_request = None
                    self.core.nudge_retry_at = now + 1.
                    self.core.replan_reason = 'No nudge found ('+why[:80]+'); following the original route'
        return True

    def plan_paused_course(self):
        """Bounded planning outside the flight lock; cancelled work cannot commit."""
        now = time.monotonic()
        with self.lock:
            pending = self.core.paused_course
            if pending is None:
                self._replan_result = None
                return False
            if self.pos is None:
                return True
            run = copy.deepcopy(pending['run'])
            guard = copy.deepcopy(pending['guard'])
            elapsed = pending.get('elapsed_s',run.elapsed)
            position = (self.pos.x,self.pos.y,self.pos.z)
            floor, ceiling = self.core._floor(), self.core._ceiling()
            depth = DashNode.depth_for_route(self, now, run.points)
        try:
            path = plan_remaining(run.points, run.times, elapsed, position,
                run.anchor, run.reference, run.radius, depth, list(guard.cells.values()),
                known_cell_m=guard.cell_m, clearance_m=guard.clearance,
                floor=floor, ceiling=ceiling)
            message = 'Route checked; waiting for a settled hover'
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            path, message = None, 'Holding for a new route: '+str(exc)
        with self.lock:
            if (self.core.paused_course is pending and self.core.mode == 'engaged'
                    and pending['reference'] == self.course.execution_reference()):
                self._replan_result = dict(pending=pending, path=path, captured_at=now,
                                          depth_age=depth.get('age_s'))
                self.core.replan_reason = message
        return True

    def _state(self, m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        o = s.get("origin_enu") or [0.0, 0.0, 0.0]
        f = Fc(time.monotonic(), bool(s.get("px4_link")), bool(s.get("armed")), bool(s.get("offboard_active")),
               bool(s.get("landed", True)), str(s.get("nav_state_name", "")), float(o[2] if o and o[2] is not None else 0.0))
        with self.lock:
            self.fc, self.fc_raw = f, s

    def _heading(self, m):
        try:
            self.heading = (time.monotonic(), json.loads(m.data))
        except (ValueError, TypeError):
            pass

    @staticmethod
    def heading_fix_enabled():
        # marker that also switches bridge.launch.py to feed PX4 the heading-fixed odometry
        return os.path.exists('/root/team_ws/claude/mavlink_usb/heading_fix_enabled')

    def request_heading_calibration(self, why):
        """Reset home / course start (grounded) restarts the heading fix's gyro-bias calibration; ARM is
        blocked until it has 10 s of still data again. Harmless when the heading fix is not running."""
        pub = getattr(self, 'pub_heading_cal', None)
        if pub is not None:
            pub.publish(String(data=why))

    def _vo(self, m):
        self.vo_state, self.vo_t = int(m.vo_state), time.monotonic()

    def _batt(self, m):
        self.batt = (time.monotonic(), float(m.voltage_v), float(m.remaining), bool(m.connected))

    def _vehicle(self, m):
        self.vehicle = (time.monotonic(), {"failsafe": bool(m.failsafe)})

    def _land(self, m):
        self.land = (time.monotonic(), bool(m.landed))

    def _arm_ack(self, m):
        if (m.command != VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                or m.target_system != 12 or m.target_component != ARM_SOURCE_COMPONENT):
            return
        cancel_launch = False
        with self.lock:
            request = self.arm_request
            if request is None or not request["pending"]:
                return
            result = int(m.result)
            label = {0: "accepted; waiting for telemetry", 1: "temporarily rejected", 2: "denied",
                     3: "unsupported", 4: "failed", 5: "in progress", 6: "cancelled"}.get(result, f"result {result}")
            # ARM and DISARM share one command ID; an ACK may belong to an earlier click.
            request["message"] = f"PX4 arm/disarm: {label}"
            request["pending"] = result in (0, 5)
            self.core.log(time.monotonic(), request["message"])
            if request.get('launch') and not request['pending'] and self.core.mode == 'engaged':
                self.core._trip(time.monotonic(), 'Takeoff cancelled: ' + request['message'])
                cancel_launch = True
        if cancel_launch:
            self.freeze_interface()

    def update_arm_request(self, now):
        request = self.arm_request
        if request is None or not request["pending"]:
            return
        if (self.fc is not None and self.fc.t >= request["sent_at"] and now - self.fc.t <= 1
                and self.fc.px4_link and self.fc.armed == (request["op"] == "arm")):
            request["message"] = "PX4 confirms " + ("ARMED" if self.fc.armed else "DISARMED")
        elif now - request["sent_at"] >= 5:
            request["message"] = f"{request['op']} not confirmed; no automatic retry"
            if request.get('launch') and self.core.mode == 'engaged':
                self.core._trip(now, 'Takeoff cancelled: ' + request['message'])
                self.freeze_interface()
        else:
            return
        request["pending"] = False
        self.core.log(now, request["message"])

    def other_setpoint_publishers(self):
        return sum(max(0, self.count_publishers("/skybeta/setpoint/" + kind) - int(self.a.live))
                   for kind in ("pose", "twist"))

    def readiness(self, now, preflight=True):
        why = []
        if preflight and getattr(self, '_ground_sync', None) is not None:
            why.append('Refreshing ground height; waiting for the flight interface')
        if self.pos is None or not self.pos.valid or now - self.pos.t > 0.5:
            why.append("PX4 local position missing or stale")
        if self.fc is None or not self.fc.px4_link or now - self.fc.t > 1:
            why.append("flight interface missing or stale")
        elif preflight and self.pos is not None and abs(self.fc.origin_z - self.pos.z) > self.core.p.floor_tol_m:
            why.append("flight interface/PX4 ground heights disagree")
        if self.land is None or now - self.land[0] > 1.5:
            why.append("PX4 landed state missing or stale")
        # PX4's default USB BATTERY_STATUS stream is 0.5 Hz.
        if self.batt is None or now - self.batt[0] > 5 or not self.batt[3]:
            why.append("flight battery missing or stale")
        elif preflight and (not math.isfinite(self.batt[2]) or self.batt[2] <= 0.2):
            why.append("battery estimate must be above 20% before arming")
        if self.vehicle is None or now - self.vehicle[0] > 2:
            why.append("PX4 status missing or stale")
        elif self.vehicle[1]["failsafe"]:
            why.append("PX4 failsafe active")
        if self.vo_t is None or now - self.vo_t > 0.5 or self.vo_state != 1:
            why.append("VIO missing, stale, or not tracking")
        if self.other_setpoint_publishers():
            why.append("another node publishes flight setpoints")
        # Gyro heading fix in the PX4 chain: arming waits for its gyro-bias calibration (drone still, motors
        # off). Preflight only, never an in-flight hold.
        if preflight and DashNode.heading_fix_enabled():
            h = getattr(self, 'heading', None)
            if h is None or now - h[0] > 2.0:
                why.append("heading fix missing or stale (/team_heading/status)")
            elif not h[1].get('calibrated'):
                left = max(0.0, float(h[1].get('calib_s', 10)) - float(h[1].get('still_s', 0)))
                why.append(f"Heading calibration: hold the drone still on the pad ({left:.0f} s left)")
            elif h[1].get('gyro_age_s') is None or h[1]['gyro_age_s'] > 0.5:
                why.append("heading fix: PX4 gyro stale")
        return why

    def freeze_interface(self):
        if self.a.live:
            if self.hold_client.service_is_ready():
                self.hold_client.call_async(Trigger.Request())
            else:
                self.get_logger().error("Hold service unavailable; pilot must take over")

    def sync_ground_reference(self, now):
        """Release an obsolete prepared height only while confirmed grounded.

        The interface refreshes its PX4 origin in Hold. A pending service must
        finish before another launch: a late Hold would cancel its setpoints.
        Called under the flight lock; never sends ARM or flight-mode requests.
        """
        if not self.a.live:
            return
        p, f, c, land = self.pos, self.fc, self.core, self.land
        grounded = (p is not None and p.valid and 0 <= now-p.t <= .5
            and all(math.isfinite(v) for v in (p.x,p.y,p.z))
            and f is not None and f.px4_link and 0 <= now-f.t <= .5
            and not f.armed and f.landed and math.isfinite(f.origin_z)
            and land is not None and land[1] and 0 <= now-land[0] <= 1.5)
        inactive = (c.mode in ('idle','failsafe') and c.queued_course is None
            and c.deferred_course is None and c.paused_course is None
            and not (c.course_run is not None and c.course_run.active)
            and not (self.arm_request and self.arm_request.get('pending')))
        pending = getattr(self, '_ground_sync', None)
        if pending is not None:
            future = pending['future']
            if not future.done():
                return  # Do not time out a service that could still execute.
            if pending['completed_at'] is None:
                try:
                    response = future.result()
                except Exception as exc:
                    if not pending.get('error_logged'):
                        c.log(now, 'Ground height refresh unconfirmed: '+str(exc))
                        pending['error_logged'] = True
                    return  # Transport failure does not prove Hold cannot arrive.
                if response is None:
                    return
                if not response.success:
                    c.log(now, 'Ground height refresh declined by the flight interface')
                    self._ground_sync = None
                    self._ground_sync_retry_at = now+1.
                    return
                pending['completed_at'] = now
            if (grounded and inactive and f.t > pending['completed_at']
                    and self.fc_raw.get('setpoint_mode') == 'hold'
                    and abs(f.origin_z-p.z) <= c.p.floor_tol_m):
                self._ground_sync = None
                c.log(now, 'Ground height synchronized with PX4')
            return
        if (not grounded or not inactive or now < getattr(self, '_ground_sync_retry_at', 0.)
                or self.fc_raw.get('altitude_reference') != 'px4'
                or abs(f.origin_z-p.z) <= c.p.floor_tol_m
                or self.other_setpoint_publishers()):
            return
        if self.hold_client.service_is_ready():
            future = self.hold_client.call_async(Trigger.Request())
            self._ground_sync = dict(future=future, completed_at=None)
            c.log(now, 'Refreshing ground height from current landed PX4 position')

    def stop_controls(self, now, client):
        """Use the same live conditions for the buttons and their API actions."""
        denied = lambda reason: dict(hold=False, land=False, reason=reason)
        p, f, c = self.pos, self.fc, self.core
        if client != c.owner:
            return denied('This tab does not have control')
        if (p is None or not p.valid or not 0 <= now-p.t <= c.p.pos_timeout_s
                or not all(math.isfinite(v) for v in (p.x,p.y,p.z,p.yaw))
                or f is None or not f.px4_link or not 0 <= now-f.t <= c.p.fc_timeout_s):
            return denied('Waiting for current aircraft position and status')
        if c.mode == 'landing':
            return denied('Landing is already in progress')
        if c.mode == 'pilot':
            return denied('Pilot takeover is latched; use the controller to land')
        # Grounded preparation can still be cancelled by Hold.
        if not f.armed:
            return dict(hold=c.mode == 'engaged', land=False, reason='Motors are disarmed')
        if not f.offboard_active:
            return denied('Use the controller: Offboard control is inactive')
        if f.landed:
            return dict(hold=c.mode == 'engaged', land=False, reason='Aircraft is on the ground')
        land = self.land
        if land is None or not 0 <= now-land[0] <= 1.5 or land[1]:
            return denied('Waiting for current airborne status')
        if (c.home is None or not all(math.isfinite(v) for v in (*c.home,c.ground_z,c.origin_z,f.origin_z))
                or abs(c.origin_z-f.origin_z) > c.p.floor_tol_m):
            return denied('Ground reference changed; use the controller to land')
        if (math.hypot(p.x-c.engage_xy[0],p.y-c.engage_xy[1]) > c.fence_radius()+c.p.fence_margin_m
                or p.z > c._ceiling()+.3):
            return denied('Aircraft is outside dashboard limits; use the controller to land')
        blockers = self.readiness(now, preflight=False) if self.a.live else []
        if blockers:
            return denied('; '.join(blockers))
        return dict(hold=p.z >= c._floor()-.02, land=True,
                    reason='' if p.z >= c._floor()-.02 else 'Below hover height; Land is available')

    def stop_action(self, now, client, op):
        """Explicit hold/descent recovery never resumes a cancelled route."""
        with self.lock:
            self._arm_after_offboard = None
            available = DashNode.stop_controls(self, now, client)
            if not available[op]:
                return False, available['reason'] or 'Stop control unavailable'
            c, p, f = self.core, self.pos, self.fc
            if not f.armed or f.landed:
                return c.command(now, client, op, p, f)
            # Retain home, ground and fence; do not reset/reanchor in flight.
            c.stop_course(now, 'Automatic course cancelled by '+op)
            c.course_run = None
            c.cmd = [p.x,p.y,p.z]
            c.target = list(c.cmd)
            c.heading.reset(p.yaw)
            c.course_yaw_rate = 0.
            c.mode, c.reason, c.was_offboard, c.last_step = 'engaged', '', True, now
            ok, message = c.command(now, client, op, p, f)
            if ok and op == 'land' and p.z <= c._floor()+.15:
                c.land_phase, c.land_t0 = 'touch', now
            c.log(now, 'Operator '+op+' after flight stop; course remains cancelled')
            return ok, message

    # ---- 10 Hz
    def _tick(self):
        now = time.monotonic()
        with self.lock:
            self.update_arm_request(now)
            pending = getattr(self, '_arm_after_offboard', None)
            if pending is not None:
                f = self.fc
                if now > pending['until']:
                    self._arm_after_offboard = None
                    message = 'PX4 did not enter Offboard within 3 s; not arming'
                    self.arm_request = dict(op='arm', pending=False, sent_at=now, message=message)
                    self.core.log(now, message)
                elif f is not None and f.offboard_active and 0 <= now-f.t <= 1:
                    self._arm_after_offboard = None
                    if (f.armed or not f.landed or self.core.mode != pending['mode']
                            or not self.core.link_alive(now)):
                        message = 'Offboard confirmed but the launch changed; not arming'
                        self.arm_request = dict(op='arm', pending=False, sent_at=now, message=message)
                        self.core.log(now, message)
                    else:
                        self.core.log(now, 'PX4 confirms Offboard')
                        DashNode.send_arm_request(self, now, 'arm', launch=pending['launch'])
            DashNode.sync_ground_reference(self, now)
            active = self.core.mode in ("engaged", "landing")
            blockers = self.readiness(now, preflight=False) if self.a.live and active else []
            if blockers:
                self.core._trip(now, "; ".join(blockers))
            run = self.core.queued_course or self.core.course_run
            if run is not None and run.active and run.reference != self.course.execution_reference():
                self.core._trip(now, 'course reference changed; course cancelled')
            pending = self.core.deferred_course
            if pending is not None and pending['reference'] != self.course.execution_reference():
                self.core._trip(now, 'course reference changed; takeoff/course cancelled')
            paused = self.core.paused_course
            if paused is not None and paused['reference'] != self.course.execution_reference():
                self.core._trip(now, 'course reference changed; replan cancelled')
            if run is not None and run.active and run.controller=='velocity_tracker':
                limit,reason=velocity_controller_limit(self.fc_raw)
                if limit is None or limit+1e-9<run.xy_speed:
                    self.core._trip(now,reason or 'Velocity interface limit changed; course cancelled')
            depth = None if self.core.p.obstacle_mode == 'shadow' else DashNode.depth_for_route(self, now)
            out = self.core.step(now, self.pos, self.fc, depth)
            if self.core.deferred_course is not None and self.core.launch_state == 'waiting_route':
                try:
                    path = self.course.execution_path(now)
                    limit, reason = 1., ''
                    if self.core.course_controller == 'velocity_tracker':
                        limit, reason = velocity_controller_limit(self.fc_raw)
                    if limit is None:
                        ok = False
                    else:
                        ok, reason = self.core.start_deferred_course(now, path, self.pos, self.fc, depth, limit)
                except (ValueError, TypeError, KeyError, OverflowError) as exc:
                    ok, reason = False, str(exc)
                if not ok:
                    self.core.launch_reason = 'Holding position: '+reason
            paused = self.core.paused_course
            result = getattr(self, '_replan_result', None)
            if paused is not None and result and result['pending'] is paused and result['path'] is not None:
                age = result['depth_age']
                # Route checked against that frame plus remembered cells; the guard
                # rechecks every command. Allow 3 s to settle and commit.
                if 0 <= now-result['captured_at'] <= 3.0:
                    limit, reason = 1., ''
                    if self.core.course_controller == 'velocity_tracker':
                        limit, reason = velocity_controller_limit(self.fc_raw)
                    if limit is not None:
                        ok, reason = self.core.resume_course(now, result['path'], self.pos, self.fc, depth, limit)
                    else:
                        ok = False
                    if not ok:
                        self.core.replan_reason = 'Holding for a new route: '+reason
                else:
                    self._replan_result = None
                    self.core.replan_reason = 'Holding; waiting for current obstacle observations'
            nudge = self.core.nudge_request
            if nudge is not None and nudge.get('path') is not None:
                limit = 1.
                if self.core.course_controller == 'velocity_tracker':
                    limit, _ = velocity_controller_limit(self.fc_raw)
                if limit is None:
                    self.core.nudge_request = None
                else:
                    self.core.apply_nudge(now, nudge['path'], limit)
            commanded_yaw = self.core.heading.yaw
            commanded_yaw_rate = getattr(self.core,'course_yaw_rate',0.)
            stopped = active and self.core.mode not in ("engaged", "landing")
            current_run = self.core.course_run
            trace = None
            if current_run is not None and current_run.controller == 'velocity_tracker':
                trace_key = (id(current_run), current_run.phase, current_run.state)
                if trace_key != getattr(self, '_last_course_trace_key', None):
                    self._last_course_trace_key = trace_key
                    trace = dict(current_run.status(),
                                 measured_heading_deg=(None if self.pos is None else
                                                       round(math.degrees(self.pos.yaw), 2)))
        if stopped:
            self.freeze_interface()
        if trace is not None:
            self.get_logger().info('velocity_tracker_transition ' + json.dumps(trace))
        if out is None:
            return
        kind, v = out
        if kind == "pose":
            m = PoseStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "odom"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(x) for x in v)
            if commanded_yaw is not None:
                (m.pose.orientation.x,m.pose.orientation.y,
                 m.pose.orientation.z,m.pose.orientation.w) = yaw_quaternion(commanded_yaw)
            else:
                m.pose.orientation.x=m.pose.orientation.y=m.pose.orientation.z=m.pose.orientation.w=0.0
            self.pub_pose.publish(m)
        else:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "odom"
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(x) for x in v)
            m.twist.angular.z = float(commanded_yaw_rate)
            self.pub_twist.publish(m)
        self.n_out += 1

    # ---- API
    def sync_course_to_home(self):
        """A known home defines the course start; no flight setpoint is changed."""
        course = getattr(self, 'course', None)
        if course is None or getattr(course, 'document', None) is None:
            return
        with self.lock:
            home = self.core.home
        if home is None:
            return
        anchor = course.snapshot().get('anchor')
        if (anchor and anchor['origin_enu'] == list(home[:3])
                and anchor['yaw_rad'] == home[3]):
            return
        course.capture_anchor(list(home[:3]), home[3])

    def snapshot(self, now, client):
        with self.lock:
            self.core.heartbeat(now, client)
            st = self.core.status(now)
            st['stop_controls'] = DashNode.stop_controls(self, now, client)
            st["arm_request"] = None if self.arm_request is None else dict(self.arm_request)
            p, f, raw = self.pos, self.fc, dict(self.fc_raw)
            run = self.core.queued_course or self.core.course_run
            execution = run if run is not None and run.active else None
        velocity_limit,velocity_reason=velocity_controller_limit(raw)
        st['course_controller'].update(
            available=velocity_limit is not None,reason=velocity_reason,
            options=[dict(id='smooth_position',label='Smooth position',speed_mps=min(MAX_COURSE_SPEED_MPS,st['params']['course_v_xy']),
                          waypoint_acceptance_m=WAYPOINT_ACCEPTANCE_M),
                     dict(id='velocity_tracker',label='Velocity tracker',
                          speed_mps=min(MAX_COURSE_SPEED_MPS,velocity_limit or 1.,st['params']['course_v_xy']))])
        st["you_own"] = st["owner"] == client
        st['control_build'] = 'adaptive-flight-v6'
        st["owner"] = None if st["owner"] is None else st["owner"][:6]
        st["live"] = self.a.live
        st["telemetry_source"] = "USB MAVLink" if self.a.usb_telemetry else "UART DDS"
        st["setpoint_topic"] = self.pose_topic
        st["pos"] = None if p is None else {"x": round(p.x, 3), "y": round(p.y, 3), "z": round(p.z, 3),
                                              "yaw_deg": round(math.degrees(p.yaw), 1), "valid": p.valid,
                                              "age_s": round(now - p.t, 2)}
        st["fc"] = None if f is None else {"px4_link": f.px4_link, "armed": f.armed, "offboard": f.offboard_active,
                                           "landed": f.landed, "nav": f.nav_state_name, "age_s": round(now - f.t, 2),
                                           "clamped": raw.get("clamped", []), "envelope": raw.get("envelope")}
        st["vio"] = None if self.vo_t is None else {"vo_state": self.vo_state, "age_s": round(now - self.vo_t, 2)}
        h = getattr(self, 'heading', None)
        st["heading_fix"] = {"enabled": DashNode.heading_fix_enabled(),
                             "status": None if h is None else dict(h[1], age_s=round(now - h[0], 2))}
        st["batt"] = None if self.batt is None else {"v": round(self.batt[1], 2),
                                                     "pct": round(100 * self.batt[2]) if math.isfinite(self.batt[2]) else None,
                                                     "connected": self.batt[3],
                                                     "age_s": round(now - self.batt[0], 1)}
        st["other_publishers"] = self.other_setpoint_publishers()
        o = raw.get("origin_enu")
        st["floor_offset"] = None if (p is None or not o or o[2] is None) else round(float(o[2]) - p.z, 2)
        preflight = bool(f is not None and not f.armed and f.landed)
        st["blockers"] = self.readiness(now, preflight=preflight)
        st["ready"] = not st["blockers"]
        st["runtime_ready"] = not self.readiness(now, preflight=False)
        st["vehicle"] = None if self.vehicle is None else self.vehicle[1]
        r = self.range_height
        st["height"] = {
            "range_m": round(r[1], 3) if r and r[2] and now - r[0] <= 0.5 else None,
            "reference": raw.get("altitude_reference", "odometry")}
        st["depth"] = self.depth_overlay.snapshot(now)
        st["course"] = self.course.snapshot(now)
        if self.core.p.obstacle_mode == 'shadow':
            st['course'].update(
                status='original' if st['course'].get('anchor') is not None else 'needs_anchor',
                reason='Depth display only; collision avoidance is off',
                preview_enabled=False, adjusted_world_enu=[], adjusted_yaw_enu_rad=[],
                adjusted_count=0, adjusted_version='shadow:'+str(st['course'].get('original_version')))
            st['course']['adjustment'].update(status='shadow',
                reason='Depth display only; collision avoidance is off', execution_supported=False)
        if execution is not None and execution.display is not None:
            # The purple line is the committed, full-resolution flight plan,
            # decimated only for display. Background previews cannot move it.
            display = execution.display
            st['course'].update(adjusted_world_enu=display['points'],
                adjusted_yaw_enu_rad=display['yaws'], adjusted_count=len(execution.points),
                adjusted_version='executing:'+display['version'])
            st['course']['adjustment'].update(status=execution.source,
                reason='Committed flight route; every command is checked against current wall observations',
                age_s=st['depth'].get('age_s'), committed=True)
        return st

    def request_course(self, now, radius, prepare=False):
        """Called under the flight lock; heavy planning stays in _course_loop."""
        if self.a.live:
            blockers = self.readiness(now, preflight=prepare)
            if blockers:
                return False, '; '.join(blockers)
        try:
            if self.core.p.obstacle_mode == 'shadow':
                depth = None
                path = self.course.original_path()
            else:
                depth = DashNode.depth_for_route(self, now)
                observations(depth)
                path = self.course.execution_path(now)
        except (ValueError, TypeError) as exc:
            return False, str(exc)
        limit = 1.
        if self.core.course_controller == 'velocity_tracker':
            limit, reason = velocity_controller_limit(getattr(self, 'fc_raw', {}))
            if limit is None:
                return False, reason
        if prepare:
            return self.core.prepare_course(now, path, radius, self.pos, self.fc, depth, limit)
        return self.core.start_course(now, path, radius, self.pos, self.fc, velocity_limit_mps=limit)

    def do_course(self, now, client, body):
        """Course setup and explicit playback through the existing flight timer."""
        if not isinstance(client, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", client):
            return False, "invalid client ID"
        op = body.get("op")
        with self.lock:
            if client != self.core.owner:
                return False, "not in control (press Take control)"
            self.core.heartbeat(now, client)
            if op == 'stop':
                self.core.stop_course(now)
                return True, 'Course stopped; holding the last commanded position'
            if op == 'options':
                return self.core.select_course_options(now, self.pos, self.fc,
                    obstacle_mode=body.get('obstacle_mode'), heading_mode=body.get('heading_mode'))
            if op == 'controller':
                controller=body.get('controller')
                if controller=='velocity_tracker':
                    limit,reason=velocity_controller_limit(getattr(self,'fc_raw',{}))
                    if limit is None:
                        return False,reason
                return self.core.select_course_controller(now,controller,self.pos,self.fc)
            if op == 'start':
                if self.core.deferred_course is not None:
                    return False, 'Takeoff already requested; the checked course starts after a settled hover'
                prepare = self.fc is not None and not self.fc.armed and self.fc.landed
                return DashNode.request_course(self, now, body.get('radius_m'), prepare=prepare)
            run = self.core.queued_course or self.core.course_run
            if (self.core.deferred_course is not None or self.core.paused_course is not None
                    or run is not None and run.active) and op in ('import','capture_anchor','preview'):
                return False, 'Stop course before changing its route or reference'
            if op == "capture_anchor":
                p, f, land = self.pos, self.fc, self.land
                if (p is None or not p.valid or now-p.t > .5
                        or not all(math.isfinite(v) for v in (p.x,p.y,p.z,p.yaw))):
                    return False, "no fresh position and heading for the course reference"
                if f is None or not f.px4_link or now-f.t > 1:
                    return False, "no fresh PX4 status for the course reference"
                if (f.armed or not f.landed or land is None
                        or now-land[0] > 1.5 or not land[1]):
                    return False, "set the course reference only while disarmed and landed"
                origin, yaw = [p.x,p.y,p.z], p.yaw
            elif op not in ("import", "preview"):
                return False, "unknown course operation"
        # File writes and trajectory math must not delay the flight timer.
        if op == "capture_anchor":
            ok, message = self.course.capture_anchor(origin, yaw)
            if ok:
                with self.lock:
                    self.core.home = (*origin, yaw)
                    self.core.course_radius = None
                    self.core.course_run = None
                    self.core.log(now, 'Home and course start set to the same position and heading')
                message = 'Home / course start set'
            return ok, message
        if op == "import":
            ok, message = self.course.import_route(body.get("trajectory"))
            if ok:
                DashNode.sync_course_to_home(self)
            return ok, message
        return self.course.configure_preview(body.get("clearance_m", .30), body.get("max_offset_m", 1.5))

    def request_offboard(self, now):
        """Ask PX4 for Offboard (DO_SET_MODE 1, 6); the USB adapter allows exactly this."""
        command = VehicleCommand()
        command.timestamp = self.get_clock().now().nanoseconds // 1000
        command.command = 176   # MAV_CMD_DO_SET_MODE
        command.param1, command.param2 = 1.0, 6.0
        command.target_system = command.source_system = 12
        command.target_component = 1
        command.source_component = ARM_SOURCE_COMPONENT
        command.from_external = True
        self.arm_pub.publish(command)

    def send_arm_request(self, now, op, launch=False, offboard_first=False):
        """One explicit operator request; caller holds the flight lock.
        offboard_first (start buttons): outside Offboard, request it and arm on confirmation.
        Manual ARM leaves the decision to PX4 unchanged."""
        if op == 'disarm':
            self._arm_after_offboard = None
        elif op == 'arm' and offboard_first and (self.fc is None or not self.fc.offboard_active):
            # PX4 will not arm in Hold indoors (no global position): switch to
            # Offboard first; _tick sends this same ARM once PX4 confirms.
            DashNode.request_offboard(self, now)
            # Wait up to 3 s for PX4 to confirm Offboard before arming.
            self._arm_after_offboard = dict(launch=launch, until=now+3., mode=self.core.mode)
            message = 'Switching PX4 to Offboard; arming when it confirms'
            self.arm_request = dict(op=op, pending=True, sent_at=now, message=message)
            self.core.log(now, message)
            return True, message
        command = VehicleCommand()
        command.timestamp = self.get_clock().now().nanoseconds // 1000
        command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        command.param1 = 1.0 if op == 'arm' else 0.0
        command.param2 = 21196.0 if op == 'disarm' else 0.0
        command.target_system = command.source_system = 12
        command.target_component = 1
        command.source_component = ARM_SOURCE_COMPONENT
        command.from_external = True
        self.arm_pub.publish(command)
        message = ('KILL sent (force disarm); waiting for PX4' if op == 'disarm' else
                   'Takeoff requested; waiting for PX4 to confirm arming' if launch else
                   'arm requested; waiting for PX4')
        self.arm_request = dict(op=op, pending=True, sent_at=now, message=message)
        if launch:
            self.arm_request['launch'] = True
        self.core.log(now, message)
        return True, message

    def launch_action(self, now, client, body):
        """Grounded setup or a single explicit takeoff action, never a queued ARM.

        Validate on a trial core before committing ownership/reset/preparation.
        Course launch retains its reference and waits for checked geometry in hover.
        Hover is explicitly selected and uses the existing manual hover limits.
        """
        setup = body.get('op') == 'launch_setup'
        flight = body.get('flight')
        if flight not in ('course', 'hover') or (setup and flight != 'course'):
            return False, 'Choose Course or Hover'
        if not isinstance(client, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', client):
            return False, 'invalid client ID'
        with self.lock:
            if getattr(self, '_ground_sync', None) is not None:
                return False, 'Refreshing ground height; wait for the flight interface'
            if not self.a.live and not setup:
                return False, 'Takeoff is disabled in preview mode'
            p, f, land = self.pos, self.fc, self.land
            if (p is None or not p.valid or not 0 <= now-p.t <= .5
                    or not all(math.isfinite(v) for v in (p.x,p.y,p.z,p.yaw))
                    or f is None or not f.px4_link or not 0 <= now-f.t <= 1.):
                return False, 'Waiting for fresh aircraft position and status'
            if not math.isfinite(f.origin_z):
                return False, 'Waiting for a valid aircraft ground height'
            if (f.armed or not f.landed or land is None or not land[1]
                    or not 0 <= now-land[0] <= 1.5):
                return False, 'Takeoff setup requires disarmed motors and the aircraft on the ground'
            if (self.arm_request and self.arm_request.get('pending') and
                    now-self.arm_request['sent_at'] < 5.):
                return False, 'Waiting for the previous arming request; it will not be retried automatically'
            if not setup:
                blockers = self.readiness(now)
                if blockers:
                    return False, '; '.join(blockers)
                # Not in Offboard: send_arm_request switches PX4 first, then arms.
                if (any(v is None or not math.isfinite(v) for v in (p.vx,p.vy,p.vz))
                        or math.hypot(p.vx,p.vy) > .15 or abs(p.vz) > .10):
                    return False, 'Waiting for fresh settled XYZ velocity before takeoff'
            trial = copy.deepcopy(self.core)
            ok, message = trial.command(now, client, 'claim', p, f)
            if not ok:
                return ok, message
            saved_home = trial.home
            ok, message = trial.command(now, client, 'reset', p, f)
            if not ok:
                return ok, message
            if setup:
                ok, message = self.course.capture_anchor([p.x,p.y,p.z], p.yaw)
                if not ok:
                    return ok, message
                self.core = trial
                self.core.log(now, 'Course start set; ready for vertical takeoff when flight checks pass')
                DashNode.request_heading_calibration(self, 'set home / course start')
            elif flight == 'hover':
                ok, message = trial.command(now, client, 'engage', p, f,
                                            alt=body.get('alt', trial.p.default_alt_m))
                if not ok:
                    return ok, message
                self.core = trial
                return DashNode.send_arm_request(self, now, 'arm', launch=True, offboard_first=True)
            else:
                # Reset the grounded fault latch without moving the course
                # anchor. A misplaced aircraft must explicitly set its start.
                trial.home = saved_home
                try:
                    depth = None if trial.p.obstacle_mode == 'shadow' else DashNode.depth_for_route(self, now)
                    path = self.course.original_path()
                except (ValueError, TypeError) as exc:
                    return False, str(exc)
                limit = 1.
                if trial.course_controller == 'velocity_tracker':
                    limit, message = velocity_controller_limit(self.fc_raw)
                    if limit is None:
                        return False, message
                ok, message = trial.prepare_takeoff_course(now, path, body.get('radius_m', 6.), p, f, depth)
                if not ok:
                    return ok, message
                self.core = trial
                return DashNode.send_arm_request(self, now, 'arm', launch=True, offboard_first=True)
        self.freeze_interface()
        if self.core.p.obstacle_mode == 'shadow':
            return True, 'Course start set. Take off will climb vertically and follow the original course.'
        return True, 'Course start set. Take off will climb vertically and check the route from a hover.'

    def do_command(self, now, client, body):
        op = body.get("op", "")
        if op in ('launch', 'launch_setup'):
            return DashNode.launch_action(self, now, client, body)
        if op in ('hold', 'land'):
            return DashNode.stop_action(self, now, client, op)
        kw = {k: v for k, v in body.items() if k in ("alt", "fwd", "right", "up", "forward", "left")}
        with self.lock:
            if op in ('arm','engage','reset') and getattr(self, '_ground_sync', None) is not None:
                return False, 'Refreshing ground height; wait for the flight interface'
            if op in ("arm", "disarm") and not self.a.live:
                return False, "arming controls are disabled in dry run"
            if op == 'arm' and type(body.get('auto_course', True)) is not bool:
                return False, 'auto_course must be true or false'
            if op == 'arm':
                # Validate the owner and normal preflight before preparing or
                # sending anything. Only this explicit API call sends ARM.
                ok, msg = self.core.command(now, client, op, pos=self.pos, fc=self.fc)
                if not ok:
                    return ok, msg
                if body.get('auto_course', True):
                    blockers = self.readiness(now)
                    if blockers:
                        return False, '; '.join(blockers)
                    if self.fc is None or self.fc.armed or not self.fc.landed:
                        return False, 'Automatic ARM requires disarmed and landed telemetry'
                    if self.core.queued_course is None:
                        ok, msg = DashNode.request_course(self, now, body.get('radius_m', 6.), prepare=True)
                        if not ok:
                            return ok, msg
                    else:
                        if self.core.queued_course.reference != self.course.execution_reference():
                            return False, 'prepared course reference changed; reset home'
                        try:
                            if self.core.p.obstacle_mode == 'shadow':
                                self.course.original_path()
                            else:
                                observations(self.depth_overlay.snapshot(now))
                                self.course.execution_path(now)
                        except (ValueError, TypeError) as exc:
                            return False, str(exc)
                elif (self.core.queued_course is not None or self.core.deferred_course is not None
                      or self.core.paused_course is not None):
                    self.core.stop_course(now, 'Automatic course cancelled before manual ARM')
            if op == "engage" and self.a.live:
                blockers = self.readiness(now)
                if blockers:
                    return False, "; ".join(blockers)
            if op == "goto" and self.a.live:
                blockers = self.readiness(now, preflight=False)
                if blockers:
                    return False, "; ".join(blockers)
            if op == "disarm":
                # emergency kill: any token holder, no control claim, no readiness or state checks
                ok, msg = True, "kill validated"
                self.core.stop_course(now, 'Course cancelled by KILL')
            else:
                ok, msg = self.core.command(now, client, op, pos=self.pos, fc=self.fc, **kw)
            if ok and op in ("arm", "disarm"):
                ok, msg = DashNode.send_arm_request(self, now, op,
                    offboard_first=(op == 'arm' and body.get('auto_course', True) is True))
        if ok and op in ("release", "reset"):
            self.freeze_interface()
        if ok and op in ("reset", "engage"):
            DashNode.sync_course_to_home(self)
        if ok and op == "reset":
            DashNode.request_heading_calibration(self, 'reset home / course start')
        self.get_logger().info(f"cmd {op} {kw} from {client[:6]}: {ok} {msg}")
        return ok, msg


def make_handler(node, token, page):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def setup(self):
            super().setup()
            self.connection.settimeout(5)
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def log_message(self, *a):
            pass

        def _json(self, code, obj):
            b = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            try:
                self.wfile.write(b)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                self.close_connection = True

        def _auth(self, q):
            return secrets.compare_digest(q.get("k", [""])[0], token)

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if not self._auth(q):
                self.close_connection = True
                return self._json(403, {"error": "add ?k=<token> (see dash_token.txt on the Jetson)"})
            if u.path in ("/", "/index.html"):
                b = page.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                return self.wfile.write(b)
            if u.path == "/api/depth.jpg":
                # Display only: the depth worker's latest left-camera + depth picture.
                path = os.path.join(os.path.dirname(HERE), "depth_overlay", "view.jpg")
                try:
                    age = time.time() - os.stat(path).st_mtime
                    with open(path, "rb") as f:
                        b = f.read(512 * 1024)
                except OSError:
                    return self._json(404, {"error": "no depth view yet"})
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Age-S", f"{age:.1f}")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                return self.wfile.write(b)
            if u.path == "/api/state":
                versions = {k: q.get(k, [""])[0] for k in ("dg", "co", "ca")}
                return self._json(200, compact_state(
                    node.snapshot(time.monotonic(), q.get("c", [""])[0]), versions))
            self._json(404, {"error": "not found"})

        def do_POST(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if not self._auth(q):
                self.close_connection = True
                return self._json(403, {"error": "bad token"})
            try:
                n = int(self.headers.get("Content-Length", "0") or 0)
                limit = 256 * 1024 if u.path == "/api/course" else 4096
                if not 0 < n <= limit:
                    raise ValueError("invalid body size")
                body = json.loads(self.rfile.read(n))
                if not isinstance(body, dict) or not isinstance(body.get("op"), str):
                    raise ValueError("expected command object")
            except (ValueError, UnicodeError):
                self.close_connection = True
                return self._json(400, {"error": "bad json"})
            if u.path == "/api/cmd":
                now = time.monotonic()
                ok, msg = node.do_command(now, str(body.get("c", "")), body)
                return self._json(200, {"ok": ok, "msg": msg, "state": compact_state(node.snapshot(now, str(body.get("c", ""))))})
            if u.path == "/api/course":
                now = time.monotonic()
                client = str(body.get("c", ""))
                ok, msg = node.do_course(now, client, body)
                return self._json(200, {"ok": ok, "msg": msg, "state": compact_state(node.snapshot(now, client))})
            self._json(404, {"error": "not found"})
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="100.64.16.93")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--live", action="store_true", help="publish /skybeta/setpoint/* (default: dry-run preview topics)")
    ap.add_argument("--usb-telemetry", action="store_true", help="use the USB MAVLink adapter's return telemetry and commands")
    ap.add_argument("--hb-timeout", type=float, default=1.0)
    ap.add_argument("--fence", type=float, default=1.5)
    defaults = DashParams()
    ap.add_argument("--horizontal-speed", type=float, default=defaults.v_xy,
                    help="horizontal position-target ramp speed in m/s")
    ap.add_argument("--course-speed", type=float, default=defaults.course_v_xy,
                    help="course horizontal position-target ramp speed in m/s")
    ap.add_argument("--min-alt", type=float, default=defaults.min_alt_m)
    ap.add_argument("--max-alt", type=float, default=defaults.max_alt_m)
    a = ap.parse_args()
    if not math.isfinite(a.horizontal_speed) or a.horizontal_speed <= 0:
        ap.error("--horizontal-speed must be finite and positive")
    if not math.isfinite(a.course_speed) or a.course_speed <= 0:
        ap.error("--course-speed must be finite and positive")
    tok_path = os.path.join(HERE, "dash_token.txt")
    if os.path.exists(tok_path):
        token = open(tok_path).read().strip()
    else:
        token = secrets.token_urlsafe(9)
        with open(tok_path, "w") as f:
            f.write(token + "\n")
        os.chmod(tok_path, 0o600)
    page = open(os.path.join(HERE, "dash.html")).read()
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)     # we own SIGINT/SIGTERM: stop HTTP too
    node = DashNode(a)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    srv = ThreadingHTTPServer((a.bind, a.port), make_handler(node, token, page))
    srv.daemon_threads = True
    import signal

    def _stop(*_):
        threading.Thread(target=srv.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    node.get_logger().info(f"dashboard on http://{a.bind}:{a.port}/?k=<token>  "
                           f"({'LIVE: ' if a.live else 'DRY-RUN: '}{node.pose_topic})")
    try:
        srv.serve_forever()
    finally:
        node.course_stop.set()
        node.course_thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

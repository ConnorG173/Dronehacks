"""bench.link - the ROS 2 side: subscribe to PX4 + VIO, publish setpoints and commands.

ROS callbacks run on a background executor thread; the test procedures run in the main
thread and read `snapshot()`.  All receive times use time.monotonic() so the main thread
and the callbacks share one clock.  PX4-bound timestamps use the system clock in
microseconds, as refnav/node.py does (PX4's uXRCE client converts it with its timesync).

Topic names follow onboard/examples/takeoff_land.py: /fmu/out/<name>[_v<N>] by the
installed px4_msgs MESSAGE_VERSION.
"""
from __future__ import annotations

import copy
import math
import re
import threading
import time


from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from .guard import State, tilt_from_q, yaw_from_q

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

CMD_NAV_LAND, CMD_DO_SET_MODE, CMD_ARM_DISARM, CMD_ACTUATOR_TEST = 21, 176, 400, 310
FORCE_DISARM_MAGIC = 21196.0        # MAV_CMD_COMPONENT_ARM_DISARM param2: disarm even in flight
PX4_MAIN_MODE_OFFBOARD = 6.0
ACTUATOR_FUNCTION_MOTOR1 = 101      # MAV_CMD_ACTUATOR_TEST param5: Motor 1 = 101, Motor 2 = 102 ...


def px4_out(name, msg_type):
    version = int(getattr(msg_type, "MESSAGE_VERSION", 0))
    return "/fmu/out/" + name + (f"_v{version}" if version else "")


class Rate:
    """Message count and rate over a sliding window."""

    def __init__(self, window_s=2.0):
        self.window, self.stamps, self.count, self.last = window_s, [], 0, None

    def hit(self, now):
        self.count += 1
        self.last = now
        self.stamps.append(now)
        while self.stamps and now - self.stamps[0] > self.window:
            self.stamps.pop(0)

    def hz(self, now):
        s = [x for x in self.stamps if now - x <= self.window]
        if len(s) < 2:
            return 0.0
        return (len(s) - 1) / max(s[-1] - s[0], 1e-6)


class Px4Link(Node):
    def __init__(self, health_topic="/estimator/healthy", vio_topic="/visual_slam/tracking/odometry"):
        super().__init__("bench")
        from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleCommand,
                                  VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
        import px4_msgs.msg as M
        self._OCM, self._TSP, self._CMD = OffboardControlMode, TrajectorySetpoint, VehicleCommand
        self.M = M
        self.lock = threading.Lock()
        self.st = State()
        self.extra = {}            # anything else worth showing: flags, battery voltage, timesync ...
        self.rates = {}
        self.last_vio = None       # (t, (x, y, z)) latest VIO odometry position, its own frame
        self.acks = []             # (t, command, result) from vehicle_command_ack, if published

        self.pub_ocm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)

        self.topics = {}
        self._sub(VehicleLocalPosition, px4_out("vehicle_local_position", VehicleLocalPosition), self._on_lpos)
        self._sub(VehicleAttitude, px4_out("vehicle_attitude", VehicleAttitude), self._on_att)
        self._sub(VehicleStatus, px4_out("vehicle_status", VehicleStatus), self._on_status)
        self._sub(VehicleLandDetected, px4_out("vehicle_land_detected", VehicleLandDetected), self._on_land)
        for name, cb in (("EstimatorStatusFlags", self._on_est_flags), ("BatteryStatus", self._on_battery),
                         ("TimesyncStatus", self._on_timesync), ("FailsafeFlags", self._on_failsafe),
                         ("VehicleCommandAck", self._on_ack), ("SensorCombined", self._on_imu)):
            t = getattr(M, name, None)
            if t is not None:
                snake = "".join("_" + c.lower() if c.isupper() else c for c in name).lstrip("_")
                self._sub(t, px4_out(snake, t), cb)
        VehicleOdometry = getattr(M, "VehicleOdometry", None)
        if VehicleOdometry is not None:      # what the EV bridge sends PX4 (an input topic, readable too)
            self._sub(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", self._on_ev, key="ev_to_px4")
        try:
            from nav_msgs.msg import Odometry
            self._sub(Odometry, vio_topic, self._on_vio, qos=qos_profile_sensor_data, key="vio_odometry")
        except ImportError:
            pass
        from std_msgs.msg import Bool
        self.health_topic = health_topic
        if health_topic:
            self._sub(Bool, health_topic, self._on_health, qos=10, key="estimator_health")

        self.sys_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.executor_ = SingleThreadedExecutor()
        self.executor_.add_node(self)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    # ---------------------------------------------------------------- plumbing
    def _sub(self, msg_type, topic, cb, qos=PX4_QOS, key=None):
        key = key or re.sub(r"_v\d+$", "", topic.rsplit("/", 1)[-1])   # vehicle_status_v1 -> vehicle_status
        self.rates[key] = Rate()
        self.topics[key] = topic

        def wrapped(m, cb=cb, key=key):
            now = time.monotonic()
            with self.lock:
                self.rates[key].hit(now)
                cb(m, now)
        self.create_subscription(msg_type, topic, wrapped, qos)

    def _spin(self):
        try:
            self.executor_.spin()
        except Exception:          # shutdown while spinning
            pass

    def close(self):
        try:
            self.executor_.shutdown()
        except Exception:
            pass
        self.destroy_node()

    def snapshot(self):
        with self.lock:
            return copy.copy(self.st), dict(self.extra)

    # ---------------------------------------------------------------- callbacks (lock held)
    def _on_lpos(self, m, now):
        st = self.st
        st.pos, st.vel, st.yaw, st.t_pos = (m.x, m.y, m.z), (m.vx, m.vy, m.vz), m.heading, now
        ya = self.extra.get("cs_yaw_align", False) and now - self.extra.get("t_flags", -1e9) <= 3.0
        heading_ok = bool(m.heading_good_for_control) or ya
        st.pos_ok = bool(m.xy_valid and m.z_valid and heading_ok
                         and all(math.isfinite(c) for c in (m.x, m.y, m.z, m.heading)))
        self.extra.update(xy_valid=bool(m.xy_valid), z_valid=bool(m.z_valid),
                          heading_good=bool(m.heading_good_for_control),
                          eph=float(getattr(m, "eph", math.nan)), epv=float(getattr(m, "epv", math.nan)))

    def _on_att(self, m, now):
        q = tuple(float(c) for c in m.q)
        self.st.tilt_deg = tilt_from_q(q)
        self.extra["att_yaw"] = yaw_from_q(q)
        self.extra["q"] = q

    def _on_status(self, m, now):
        st = self.st
        st.armed, st.nav_state, st.t_status = int(m.arming_state) == 2, int(m.nav_state), now
        self.extra.update(preflight_ok=bool(getattr(m, "pre_flight_checks_pass", False)),
                          failsafe=bool(getattr(m, "failsafe", False)))

    def _on_land(self, m, now):
        self.st.landed = bool(m.landed)

    def _on_est_flags(self, m, now):
        self.extra["t_flags"] = now
        for f in ("cs_tilt_align", "cs_yaw_align", "cs_ev_pos", "cs_ev_yaw", "cs_ev_hgt", "cs_ev_vel",
                  "cs_baro_hgt", "cs_mag_hdg", "cs_mag_3d", "cs_gps", "cs_inertial_dead_reckoning"):
            if hasattr(m, f):
                self.extra[f] = bool(getattr(m, f))

    def _on_battery(self, m, now):
        rem = float(getattr(m, "remaining", -1.0))
        self.st.battery_frac = rem if math.isfinite(rem) else None
        self.extra["battery_v"] = float(getattr(m, "voltage_v", math.nan))

    def _on_timesync(self, m, now):
        self.extra["rtt_ms"] = float(getattr(m, "round_trip_time", math.nan)) / 1000.0

    def _on_failsafe(self, m, now):
        active = []
        for name in dir(m):
            if name.startswith("_") or name in ("timestamp", "mode_req_angular_velocity"):
                continue
            try:
                val = getattr(m, name)
            except Exception:
                continue
            if isinstance(val, bool) and val and not name.startswith("mode_req"):
                active.append(name)
        self.extra["failsafe_flags"] = active

    def _on_ack(self, m, now):
        self.acks.append((now, int(m.command), int(m.result)))
        del self.acks[:-50]

    def _on_imu(self, m, now):
        a = [float(c) for c in m.accelerometer_m_s2]
        self.extra["accel"] = a

    def _on_ev(self, m, now):
        self.extra["ev_pos"] = tuple(float(c) for c in m.position)
        self.extra["ev_reset_counter"] = int(getattr(m, "reset_counter", 0))

    def _on_vio(self, m, now):
        p = m.pose.pose.position
        self.last_vio = (now, (p.x, p.y, p.z))

    def _on_health(self, m, now):
        self.st.health, self.st.t_health = bool(m.data), now

    # ---------------------------------------------------------------- outputs
    def _us(self):
        return self.sys_clock.now().nanoseconds // 1000

    def send_setpoint(self, sp):
        o = self._OCM()
        o.timestamp = self._us()
        o.position = True              # PX4 also uses the finite velocity/acceleration as feedforward
        self.pub_ocm.publish(o)
        s = self._TSP()
        s.timestamp = o.timestamp
        s.position = [float(c) for c in sp.p]
        s.velocity = [float(c) for c in sp.v]
        s.acceleration = [float(c) for c in sp.a]
        s.yaw = float(sp.yaw)
        s.yawspeed = float(sp.yaw_rate)
        self.pub_sp.publish(s)

    def command(self, cmd, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        c = self._CMD()
        c.timestamp = self._us()
        c.command = int(cmd)
        c.param1, c.param2, c.param3, c.param4 = float(p1), float(p2), float(p3), float(p4)
        c.param5, c.param6, c.param7 = float(p5), float(p6), float(p7)
        c.target_system = c.target_component = c.source_system = c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)

    def do(self, what):
        if what == "offboard":
            self.command(CMD_DO_SET_MODE, 1.0, PX4_MAIN_MODE_OFFBOARD)
        elif what == "arm":
            self.command(CMD_ARM_DISARM, 1.0)
        elif what == "disarm":
            self.command(CMD_ARM_DISARM, 0.0)            # PX4 refuses this in the air
        elif what == "kill":
            self.command(CMD_ARM_DISARM, 0.0, FORCE_DISARM_MAGIC)
        elif what == "land":
            self.command(CMD_NAV_LAND)
        else:
            raise ValueError(what)

    def actuator_test(self, motor, value, timeout_s):
        """Spin one motor (1-based) at `value` (0 = idle spin .. 1 = full) for timeout_s (PX4 caps it).
        Needs COM_MOT_TEST_EN=1 and a disarmed vehicle.  value NaN stops it."""
        self.command(CMD_ACTUATOR_TEST, value, timeout_s, 0.0, 0.0, ACTUATOR_FUNCTION_MOTOR1 + motor - 1)

    def ack_since(self, t, command):
        """Results of acks for `command` received after time t (MAV_RESULT: 0 accepted, 1 temporarily
        rejected, 2 denied, 3 unsupported, 4 failed, 5 in progress)."""
        with self.lock:
            return [r for (ta, c, r) in self.acks if ta >= t and c == command]

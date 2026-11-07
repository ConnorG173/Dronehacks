"""The path follower: one state machine for the simulator and the real drone. Pure numpy.

    cmd = follower.step(now, vehicle_state)

`now` is the adapter's node clock in seconds (sim time in the simulator, system time on the
Jetson). Message receipt times in VehicleState must be on the same clock. Everything in
VehicleState and Command is in the vehicle's LOCAL ENU frame (see frames.py); the follower owns
T = course <- local, set once on the pad.

States
  WAIT     wait for a fresh position, valid estimator, fresh status, landed + disarmed; pad init (T)
  READY    stream "hold where you are". self_arm: after prime_s request Offboard + arm once, give up
           after arm_timeout_s, abort if the vehicle arms before our request. Otherwise wait for the
           pilot. Armed + Offboard -> TAKEOFF
  TAKEOFF  command the path start (hover point); within takeoff_radius_m for takeoff_settle_s -> FOLLOW
  FOLLOW   PathClock: governed path time, p/v/a feedforward and the on-path lead point q
  FINISH   path complete: hold the goal; land_at_end requests NAV_LAND every land_resend_s
  HOLD     fault: hold the last on-path point p(tau) (never the lead point); optional land request
           after land_after_hold_s
  LANDING  PX4 reports AUTO_LAND after our request: stream nothing, wait for landed + disarmed
  DONE     landed and disarmed after our land request
  STOPPED  yielded: publish nothing, ever again (restart the node for a new run)

Faults (-> HOLD from TAKEOFF/FOLLOW/FINISH, -> STOPPED from READY): stale position, local
position jump > jump_m, estimator not ok, vehicle outside the course fence.
Offboard (or arming) lost while flying: Config.on_offboard_lost, "hold" keeps streaming the hold
point (real drone, the previous path_streamer behaviour), "stop" -> STOPPED (simulator, like
onboard/examples/takeoff_land.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .clock import PathClock
from .frames import Transform2D, clamp_to_fence, inside_fence, pad_init, wrap
from .path import CoursePath, time_scale_for

WAIT, READY, TAKEOFF, FOLLOW, FINISH, HOLD, LANDING, DONE, STOPPED = (
    "WAIT", "READY", "TAKEOFF", "FOLLOW", "FINISH", "HOLD", "LANDING", "DONE", "STOPPED")
FLYING = (TAKEOFF, FOLLOW, FINISH, HOLD)


@dataclass
class Config:
    # course (planner/config/course_v3.yaml StartPos / StartHeadingDeg; cage +-3.049 m)
    pad_xy: tuple = (-2.5146, 0.9144)
    pad_yaw: float = 0.0                       # rad; StartHeadingDeg 0 is ASSUMED in course_v3.yaml
    cage_half_m: float = 3.049
    fence_margin_m: float = 0.3                # setpoints are clamped to the cage minus this
    fence_fault_margin_m: float = 0.15         # vehicle beyond the cage minus this -> HOLD. Smaller than
                                               # fence_margin_m: course_v3's path reaches 1.4 mm from the
                                               # 0.3 m line at M1/M3, so equal margins fault on any overshoot
    # path playback
    max_path_speed_mps: float = 1.0            # the path is played slower to stay under this
    mpc_xy_p: float = 0.95                     # the drone's MPC_XY_P; the lead point is 1/mpc_xy_p s ahead
    lag_ok_m: float = 0.3                      # path clock at full rate below this lag
    max_tracking_err_m: float = 0.6            # path clock stopped at or beyond this lag
    clock_slew_per_s: float = 1.0              # path clock rate changes by at most this per s
    max_dt_s: float = 0.25                     # a longer gap between steps advances the clock by this
    takeoff_radius_m: float = 0.25
    takeoff_settle_s: float = 2.0
    command_yaw: bool = True                   # False: yaw None (do not command heading)
    # faults
    jump_m: float = 0.5                        # local position step that counts as an estimator jump
    pose_timeout_s: float = 0.5
    status_timeout_s: float = 1.0
    # pad init (None switches a check off)
    max_pad_offset_m: Optional[float] = 0.15   # local position must be near its origin
    max_pad_yaw_deg: Optional[float] = 5.0     # local yaw must be near zero
    max_tilt_diff_deg: Optional[float] = 3.0   # local-estimate tilt vs IMU tilt
    z_from_pad: bool = False                   # True: z0 = local z on the pad; False: local z is height
    # environment differences: who arms and how a run ends (each adapter sets these)
    self_arm: bool = False                     # request Offboard + arm ourselves (simulator only)
    prime_s: float = 2.0                       # setpoint stream before the request (PX4 needs > 1 s)
    arm_timeout_s: float = 8.0
    wait_timeout_s: Optional[float] = None     # WAIT / READY before the arm request: give up after
    land_at_end: bool = False                  # request NAV_LAND at the end of the path
    land_after_hold_s: Optional[float] = None  # request NAV_LAND this long after a fault HOLD
    land_resend_s: float = 1.0
    land_confirm_s: float = 5.0                # disarmed without a fresh landed report: give up after
    on_offboard_lost: str = "hold"             # "hold" or "stop"
    dry_run: bool = False                      # pretend Offboard + armed + perfect tracking (preview)

    def __post_init__(self):
        if self.on_offboard_lost not in ("hold", "stop"):
            raise ValueError(f"on_offboard_lost must be 'hold' or 'stop', not {self.on_offboard_lost!r}")


@dataclass
class VehicleState:
    """What the adapter last received. Stamps are receipt times on the adapter's node clock (s);
    None = never received."""
    pose_stamp: Optional[float] = None
    pos: Optional[np.ndarray] = None           # (3,) local ENU, z up
    yaw: Optional[float] = None                # local ENU heading, CCW
    estimate_ok: bool = False                  # estimator says the pose is usable
    status_stamp: Optional[float] = None
    armed: bool = False
    offboard: bool = False
    auto_land: bool = False                    # PX4 is in its automatic landing mode
    landed_stamp: Optional[float] = None
    landed: bool = False
    local_tilt_deg: Optional[float] = None     # tilt of the local estimate (odom), pad check
    imu_stamp: Optional[float] = None
    imu_tilt_deg: Optional[float] = None       # IMU tilt (PX4 attitude), pad check


@dataclass
class Command:
    """What to send this step, local ENU. publish False: send no setpoint at all."""
    mode: str
    reason: str
    publish: bool = False
    p: np.ndarray = field(default_factory=lambda: np.zeros(3))   # where the vehicle should be now
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))   # velocity feedforward
    a: np.ndarray = field(default_factory=lambda: np.zeros(3))   # acceleration feedforward
    yaw: Optional[float] = None                                  # heading at p (None: don't command)
    yaw_rate: float = 0.0
    q: np.ndarray = field(default_factory=lambda: np.zeros(3))   # lead point (position-only interfaces)
    q_yaw: Optional[float] = None                                # heading at q
    request_offboard_arm: bool = False
    request_land: bool = False


def _fresh(stamp, now, max_age):
    return stamp is not None and 0.0 <= now - stamp <= max_age


class Follower:
    def __init__(self, path: CoursePath, cfg: Config = None):
        self.path, self.cfg = path, cfg or Config()
        self.scale = time_scale_for(path, self.cfg.max_path_speed_mps)
        self.mode, self.reason = WAIT, "starting"
        self.T: Optional[Transform2D] = None
        self.z0 = 0.0
        self.clock: Optional[PathClock] = None
        self.sp = None                  # last clock Setpoint (FOLLOW)
        self.hold = None                # (xy, z, yaw) course: what HOLD sends, always on the path
        self.sent = None                # (xyz, yaw) course: last setpoint published (lead point)
        self.err = None
        self.vehicle = None             # last vehicle position, course frame
        self.last_now = None
        self.t_start = None
        self.ready_t0 = None
        self.arm_requested_at = None
        self.settle_t0 = None
        self.hold_t0 = None
        self.land_requested_at = None
        self.disarmed_at = None
        self.last_xy = None

    # ------------------------------------------------------------------ helpers
    def _set(self, mode, reason):
        self.mode, self.reason = mode, reason

    def _idle(self):
        return Command(self.mode, self.reason)

    def _stop(self, why):
        self._set(STOPPED, why)
        return self._idle()

    def _fault(self, why, now):
        if self.mode in (TAKEOFF, FOLLOW, FINISH):
            self._set(HOLD, why)
            self.hold_t0 = now
        elif self.mode == READY:
            self._set(STOPPED, why)
        elif self.mode in (WAIT, HOLD):
            self.reason = why if self.mode == WAIT else self.reason

    def vehicle_course(self, s: VehicleState):
        return np.r_[self.T.to_course(s.pos[:2]), s.pos[2] - self.z0]

    def _course_cmd(self, p, yaw, v=None, a=None, yaw_rate=0.0, q=None, q_yaw=None):
        """Course-frame setpoint -> fence clamp -> local Command. A clamped p drops v and a."""
        c = self.cfg
        p = np.asarray(p, float)
        q = p if q is None else np.asarray(q, float)
        v = np.zeros(3) if v is None else np.asarray(v, float)
        a = np.zeros(3) if a is None else np.asarray(a, float)
        pxy = clamp_to_fence(p[:2], c.cage_half_m, c.fence_margin_m)
        if not np.allclose(pxy, p[:2]):
            v, a = np.zeros(3), np.zeros(3)
        qxy = clamp_to_fence(q[:2], c.cage_half_m, c.fence_margin_m)
        q_yaw = yaw if q_yaw is None else q_yaw
        self.sent = (np.r_[qxy, q[2]], q_yaw)
        T = self.T
        return Command(
            self.mode, self.reason, publish=True,
            p=np.r_[T.to_local(pxy), p[2] + self.z0], v=T.vec_to_local(v), a=T.vec_to_local(a),
            yaw=wrap(yaw - T.yaw) if c.command_yaw else None, yaw_rate=float(yaw_rate),
            q=np.r_[T.to_local(qxy), q[2] + self.z0],
            q_yaw=wrap(q_yaw - T.yaw) if c.command_yaw else None)

    def _hold_cmd(self):
        xy, z, yaw = self.hold
        return self._course_cmd(np.r_[xy, z], yaw)

    def _land_due(self, now):
        due = self.land_requested_at is None or now - self.land_requested_at >= self.cfg.land_resend_s
        if due:
            self.land_requested_at = now
        return due

    # ------------------------------------------------------------------ the step
    def step(self, now, s: VehicleState) -> Command:
        c, now = self.cfg, float(now)
        if self.mode in (DONE, STOPPED):
            return self._idle()
        if self.last_now is not None and now < self.last_now:
            return self._stop("clock moved backwards; yielding")
        dt = 0.0 if self.last_now is None else min(now - self.last_now, c.max_dt_s)
        self.last_now = now
        if self.t_start is None:
            self.t_start = now

        pose_ok = (_fresh(s.pose_stamp, now, c.pose_timeout_s) and s.pos is not None
                   and s.yaw is not None and bool(np.isfinite(s.pos).all()) and math.isfinite(s.yaw))
        status_ok = _fresh(s.status_stamp, now, c.status_timeout_s)
        armed = c.dry_run or (status_ok and s.armed)
        offboard = c.dry_run or (status_ok and s.offboard)

        # estimator faults, checked every step
        if pose_ok:
            xy = np.asarray(s.pos[:2], float)
            if self.last_xy is not None and np.linalg.norm(xy - self.last_xy) > c.jump_m:
                # HALF-ASSED: a jump only holds (pilot takes over); the brief's "inflate T,
                # re-acquire with N consistent fixes" is not implemented.
                self._fault(f"local position jumped {np.linalg.norm(xy - self.last_xy):.2f} m in one step", now)
            self.last_xy = xy.copy()
            if not s.estimate_ok and self.mode in FLYING + (READY,):
                self._fault("estimator reports the position is not usable", now)
            if self.T is not None:
                self.vehicle = self.vehicle_course(s)
        if self.mode in (DONE, STOPPED):
            return self._idle()

        if self.mode == WAIT:
            return self._wait(now, s, pose_ok, status_ok)
        if self.mode == READY:
            cmd = self._ready(now, s, pose_ok, status_ok, armed, offboard)
            if cmd is not None:
                return cmd
        if self.mode == LANDING:
            return self._landing(now, s, status_ok)

        # our land request was accepted: PX4 lands by itself, we stop streaming
        if (self.mode in (FINISH, HOLD) and self.land_requested_at is not None
                and status_ok and s.auto_land):
            self._set(LANDING, "PX4 is landing; setpoint stream stopped")
            return self._idle()
        if self.mode in FLYING and not (offboard and armed):
            if c.on_offboard_lost == "stop":
                return self._stop("Offboard or arming lost; yielding to PX4 / the pilot")
            if self.mode != HOLD:
                self._set(HOLD, "Offboard ended; holding (HALF-ASSED: restart the node for a new run)")
                self.hold_t0 = now

        if self.mode == TAKEOFF:
            return self._takeoff(now, s, pose_ok)
        if self.mode == FOLLOW:
            return self._follow(now, s, pose_ok, dt)
        if self.mode == FINISH:
            cmd = self._hold_cmd()
            cmd.request_land = c.land_at_end and self._land_due(now)
            return cmd
        # HOLD
        cmd = self._hold_cmd()
        if c.land_after_hold_s is not None and now - self.hold_t0 >= c.land_after_hold_s:
            cmd.request_land = self._land_due(now)
        return cmd

    # ------------------------------------------------------------------ states
    def _wait(self, now, s, pose_ok, status_ok):
        c = self.cfg
        if c.wait_timeout_s is not None and now - self.t_start > c.wait_timeout_s:
            return self._stop("preflight timeout: no valid pad init; nothing was requested")
        if not pose_ok:
            self.reason = "waiting for a fresh position"
        elif not s.estimate_ok:
            self.reason = "waiting for the estimator (position / heading not valid)"
        elif not status_ok:
            self.reason = "waiting for vehicle status"
        elif s.armed or not (s.landed and _fresh(s.landed_stamp, now, c.status_timeout_s)):
            # HALF-ASSED: initialisation only on the pad (landed, disarmed); no in-air (re)init.
            self.reason = "pad init needs the vehicle landed and disarmed"
        else:
            imu = s.imu_tilt_deg if _fresh(s.imu_stamp, now, c.pose_timeout_s) else None
            T, why = pad_init(s.pos[:2], s.yaw, s.local_tilt_deg, imu, c.pad_xy, c.pad_yaw,
                              c.max_pad_offset_m, c.max_pad_yaw_deg, c.max_tilt_diff_deg)
            self.reason = why
            if T is not None:
                # HALF-ASSED: T is fixed from here on. No localisation fixes are fused, so drift
                # of the local estimate goes uncorrected. The wide pad registration is not done.
                self.T = T
                self.z0 = float(s.pos[2]) if c.z_from_pad else 0.0
                self._set(READY, "pad init done; waiting for Offboard"
                          + (" (we request it)" if c.self_arm else " (pilot)"))
                self.ready_t0 = now
        return self._idle()

    def _ready(self, now, s, pose_ok, status_ok, armed, offboard):
        """Returns a Command, or None after switching to TAKEOFF (handled in the same step)."""
        c = self.cfg
        if not (pose_ok and status_ok):
            self.ready_t0 = None if self.arm_requested_at is None else self.ready_t0
            self.reason = "position or status stale; not streaming"
            return self._idle()
        if armed and offboard:
            self._set(TAKEOFF, "climbing to the path start")
            self.settle_t0 = None
            self.hold = self.path.at(0.0)
            return None
        if c.self_arm:
            if self.arm_requested_at is None and s.armed:
                return self._stop("vehicle armed before our request; yielding")
            if self.arm_requested_at is None and c.wait_timeout_s is not None \
                    and now - self.t_start > c.wait_timeout_s:
                return self._stop("preflight timeout before the arm request")
            if self.arm_requested_at is not None and now - self.arm_requested_at > c.arm_timeout_s:
                return self._stop("Offboard / arm request timed out")
        # like square_mission / takeoff_land: stream where we are, so Offboard starts with a hold
        cmd = Command(self.mode, self.reason, publish=True, p=np.asarray(s.pos, float).copy(),
                      yaw=float(s.yaw) if c.command_yaw else None, q=np.asarray(s.pos, float).copy(),
                      q_yaw=float(s.yaw) if c.command_yaw else None)
        self.sent = None
        if c.self_arm:
            if self.ready_t0 is None:
                self.ready_t0 = now
            if self.arm_requested_at is None and now - self.ready_t0 >= c.prime_s:
                self.arm_requested_at = now
                self.reason = "Offboard + arm requested"
                cmd.reason, cmd.request_offboard_arm = self.reason, True
        return cmd

    def _takeoff(self, now, s, pose_ok):
        c = self.cfg
        xy, z, yaw = self.path.at(0.0)
        self.hold = (xy, z, yaw)
        if not (pose_ok or c.dry_run):
            self._fault("position stale", now)
            return self._hold_cmd()
        veh = np.r_[xy, z] if c.dry_run else self.vehicle
        self.err = float(np.linalg.norm(veh - np.r_[xy, z]))
        if self.err > c.takeoff_radius_m:
            self.settle_t0 = None
        elif self.settle_t0 is None:
            self.settle_t0 = now
        elif now - self.settle_t0 > c.takeoff_settle_s:
            self.clock = PathClock(self.path, self.scale, lead_s=1.0 / c.mpc_xy_p, e_ok_m=c.lag_ok_m,
                                   e_stop_m=c.max_tracking_err_m, r_slew=c.clock_slew_per_s)
            self._set(FOLLOW, "following the path")
        return self._course_cmd(np.r_[xy, z], yaw)

    def _follow(self, now, s, pose_ok, dt):
        c = self.cfg
        if c.dry_run:
            xy, z, _ = self.path.at(self.clock.tau)
            veh = np.r_[xy, z]
        else:
            if not pose_ok:
                self._fault("position stale", now)
                return self._hold_cmd()
            veh = self.vehicle
            if not inside_fence(veh[:2], c.cage_half_m, c.fence_fault_margin_m):
                self._fault(f"vehicle outside the course fence at ({veh[0]:.2f}, {veh[1]:.2f})", now)
                return self._hold_cmd()
        sp = self.sp = self.clock.step(dt, veh)
        self.err = sp.err
        yaw_p = self.path.at(sp.tau)[2]
        self.hold = (sp.p[:2], sp.p[2], yaw_p)     # a fault holds p(tau), never the lead point
        if self.clock.done:
            self._set(FINISH, "path complete; holding the goal"
                      + ("; landing requested" if c.land_at_end else "; pilot lands"))
        # HALF-ASSED (real drone): position only, so the lead point q stands in for the
        # feedforward; it assumes the drone's MPC_XY_P (Config.mpc_xy_p).
        return self._course_cmd(sp.p, yaw_p, v=sp.v, a=sp.a,
                                yaw_rate=self.path.yaw_rate_at(sp.tau) * sp.rate, q=sp.q, q_yaw=sp.yaw)

    def _landing(self, now, s, status_ok):
        c = self.cfg
        if not status_ok:
            return self._stop("status lost while landing; check the vehicle")
        if s.armed:
            self.disarmed_at = None
            if not s.auto_land:
                return self._stop("flight mode changed during the automatic landing; yielding")
            return self._idle()
        if s.landed and _fresh(s.landed_stamp, now, c.status_timeout_s):
            self._set(DONE, "landed and disarmed")
            return self._idle()
        if self.disarmed_at is None:
            self.disarmed_at = now
        if now - self.disarmed_at >= c.land_confirm_s:
            return self._stop("disarmed without a fresh landed report; inspect the vehicle")
        return self._idle()

    # ------------------------------------------------------------------ telemetry
    def status(self):
        r = lambda v, n=3: None if v is None else round(float(v), n)
        st = {"mode": self.mode, "reason": self.reason,
              "T": None if self.T is None else [r(self.T.x), r(self.T.y), r(math.degrees(self.T.yaw), 2)],
              "z0": r(self.z0), "path_s": r(self.path.duration, 2), "time_scale": r(self.scale),
              "tau": None if self.clock is None else r(self.clock.tau, 2),
              "clock_rate": None if self.sp is None else r(self.sp.rate),
              "clock_stalled": None if self.clock is None else bool(self.clock.stalled),
              "lag_m": None if self.sp is None else r(self.sp.lag),
              "lead_m": None if self.sp is None else r(np.linalg.norm(self.sp.q - self.sp.p)),
              "err_m": r(self.err), "dry_run": self.cfg.dry_run}
        if self.vehicle is not None:
            st["vehicle_course"] = [r(v) for v in self.vehicle]
        if self.sent is not None:
            st["setpoint_course"] = [r(v) for v in self.sent[0]]
        return st

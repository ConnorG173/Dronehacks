"""Mission state machine: idle -> takeoff -> course -> landing -> landed -> done.

Pure logic, no ROS. One call per control tick returns the velocity command (map
ENU) and yaw rate. The output is always a velocity: this node never sends a pose,
because a pose becomes the flight interface's hold point and a lapsed twist stream
would fly the vehicle back to it (docs/proposals/path_following.md section 1).
"""
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .tracker import Tracker, TrackerParams, hold_velocity

IDLE, TAKEOFF, COURSE, LANDING, LANDED, DONE, ABORT = (
    'idle', 'takeoff', 'course', 'landing', 'landed', 'done', 'abort')
FLYING = (TAKEOFF, COURSE, LANDING, LANDED)


@dataclass
class MissionParams:
    takeoff_vz: float = 1.0        # m/s max climb
    takeoff_vxy: float = 0.3       # m/s max horizontal correction during takeoff
    settle_tol: float = 0.12       # m position error to call the hover settled
    settle_speed: float = 0.15     # m/s
    settle_time: float = 0.3       # s inside tolerance before the course starts
    settle_max: float = 4.0        # s after reaching height: start anyway
    course_start_dz: float = 0.15   # m: >0 starts the course once this close to hover height (no settle)
    arrive_tol: float = 0.20       # m at the goal before landing
    finish_wait_max: float = 4.0   # s after the clock ends: land anyway
    land_vz: float = 0.8           # m/s descent
    land_vz_slow: float = 0.3      # m/s final descent
    land_slow_height: float = 0.25  # m above the pad: switch to slow
    land_vxy: float = 0.3
    odom_timeout: float = 0.25     # s: older pose -> zero velocity
    warn_scale: float = 0.5        # speed scale on localization WARN


@dataclass
class Inputs:
    now: float
    offboard: bool
    armed: bool
    landed: Optional[bool]
    pos: Optional[np.ndarray]      # map ENU, latest (already latency compensated)
    vel: Optional[np.ndarray]      # map ENU
    yaw: Optional[float]           # map
    pose_age: float = 0.0
    loc_level: str = 'OK'          # OK | WARN | ERROR
    clearance_scale: float = 1.0


@dataclass
class Output:
    v: np.ndarray
    yaw_rate: float
    state: str
    note: str = ''
    debug: dict = field(default_factory=dict)


class Mission:

    def __init__(self, traj, pad_z, tracker_params=None, params=None):
        self.traj = traj
        self.pad_z = float(pad_z)      # map z of the ground under the goal
        self.p = params or MissionParams()
        self.tracker = Tracker(traj, tracker_params or TrackerParams())
        self.state = IDLE
        self.t_state = None
        self.t_settled = None
        self.t_height = None
        self.t_finished = None
        self.t_touch = None
        self.events = []               # (time, state) transitions for logs/metrics

    def _go(self, now, state, note=''):
        self.state = state
        self.t_state = now
        self.events.append((now, state, note))

    def step(self, dt, s):
        if self.t_state is None:
            self.t_state = s.now

        if self.state in FLYING and not s.offboard:
            if self.state == LANDED or (self.state == LANDING and (s.landed or not s.armed)):
                self._go(s.now, DONE, 'offboard ended after touchdown')
            else:
                self._go(s.now, ABORT, 'offboard lost in flight')
            return Output(np.zeros(3), 0.0, self.state)

        if self.state in (DONE, ABORT):
            return self._zero()

        if self.state == IDLE:
            if s.offboard and s.armed and s.pos is not None:
                self._go(s.now, TAKEOFF)
            else:
                return self._zero()

        # flying states need a fresh pose
        if s.pos is None or s.pose_age > self.p.odom_timeout:
            return Output(np.zeros(3), 0.0, self.state, 'stale pose: zero velocity')
        if s.loc_level == 'ERROR':
            return Output(np.zeros(3), 0.0, self.state, 'localization ERROR: holding')

        pos = np.asarray(s.pos, float)
        vel = np.zeros(3) if s.vel is None else np.asarray(s.vel, float)
        yaw = 0.0 if s.yaw is None else float(s.yaw)
        hover = self.traj.p[0]
        p = self.p

        if self.state == TAKEOFF:
            v = hold_velocity(pos, hover, kp=1.2, v_xy_max=p.takeoff_vxy, v_z_max=p.takeoff_vz)
            yr = self._yaw_hold(yaw, self.traj.yaw[0])
            err = float(np.linalg.norm(hover - pos))
            if abs(hover[2] - pos[2]) < p.settle_tol and self.t_height is None:
                self.t_height = s.now
            ok = err < p.settle_tol and float(np.linalg.norm(vel)) < p.settle_speed
            if ok:
                self.t_settled = self.t_settled or s.now
            else:
                self.t_settled = None
            early = p.course_start_dz > 0.0 and abs(hover[2] - pos[2]) < p.course_start_dz \
                and float(np.linalg.norm((hover - pos)[:2])) < p.settle_tol
            if early or (self.t_settled is not None and s.now - self.t_settled >= p.settle_time) or \
               (self.t_height is not None and s.now - self.t_height >= p.settle_max):
                # the plan starts at rest, so the clock can start at full rate
                self.tracker.reset(tau=0.0, taudot=self.tracker.p.taudot_max)
                self._go(s.now, COURSE)
            return Output(v, yr, self.state)

        if self.state == COURSE:
            scale = s.clearance_scale * (p.warn_scale if s.loc_level == 'WARN' else 1.0)
            v, yr = self.tracker.step(dt, pos, yaw, speed_scale=scale)
            if self.tracker.finished:
                self.t_finished = self.t_finished or s.now
                at_goal = float(np.linalg.norm(self.traj.p[-1] - pos)) < p.arrive_tol
                if at_goal or s.now - self.t_finished > p.finish_wait_max:
                    self._go(s.now, LANDING)
            return Output(v, yr, self.state, debug=dict(self.tracker.last))

        goal = self.traj.p[-1]
        if self.state == LANDING:
            v = hold_velocity(pos, goal, kp=1.2, v_xy_max=p.land_vxy, v_z_max=0.0)
            slow = pos[2] - self.pad_z < p.land_slow_height
            v[2] = -(p.land_vz_slow if slow else p.land_vz)
            if s.landed:
                self.t_touch = s.now
                self._go(s.now, LANDED)
            return Output(v, self._yaw_hold(yaw, self.traj.yaw[-1]), self.state)

        if self.state == LANDED:
            if not s.armed:
                self._go(s.now, DONE, 'disarmed')
                return self._zero()
            # keep pressing down until PX4 disarms (COM_DISARM_LAND)
            return Output(np.array([0.0, 0.0, -p.land_vz_slow]), 0.0, self.state)

        return self._zero()

    def _zero(self):
        return Output(np.zeros(3), 0.0, self.state)

    def _yaw_hold(self, yaw, target, k=1.5, rmax=1.0):
        from .frames import wrap_pi
        return float(np.clip(k * wrap_pi(target - yaw), -rmax, rmax))

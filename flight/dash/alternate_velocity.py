"""Dashboard adapter for the upstream inverse-lag velocity tracker.

Input geometry is an already validated/retimed CourseRun in world ENU. This
module does not arm, land, publish, read hardware, or load a different route.
Camera yaw follows the geometric path tangent. A velocity setpoint can point
sideways/backward to correct position or brake PX4's velocity loop: that is
not a new camera heading. In particular, atan2 of a near-zero correction must
never replace the initial tangent. tau_v=.55s and delay=.10s are assumptions,
not measurements of this vehicle. Initial alignment uses an absolute pose/yaw
hold, supplied by the caller through position_hold and commanded_heading.

The caller still owns authority, freshness, native failsafes and the actual
position breach/cross-track abort. This adapter bounds each proposed velocity
and its one-step endpoint against the supplied circle/altitude envelope.
Heading pacing uses the path tangent. A velocity envelope is not obstacle avoidance.
"""
import copy
import math

import numpy as np

from alt_tracker.pathfile import Trajectory
from velocity_tracker import Tracker, TrackerParams
from path_heading import tangent_yaws, wrap_angle
from yaw_tracker import YawTracker
from course_follow import end_reached


MAX_YAW_RATE = math.radians(90.)
MAX_YAW_ACCEL = math.radians(180.)
ENTER_HEADING = math.radians(5.)
EXIT_HEADING = math.radians(10.)
ALIGN_YAW_RATE = math.radians(45.)
ALIGN_YAW_ACCEL = math.radians(90.)
SETTLE_YAW_RATE = math.radians(5.)
ALIGN_TIMEOUT_S = 12.
MAX_LINEAR_ACCEL = 1.0
MAX_VERTICAL_ACCEL = .5


class AlternateVelocityRun:
    controller = 'velocity_tracker'

    def __init__(self, validated, *, position_hold_supported=False):
        if not position_hold_supported:
            raise ValueError('Velocity tracker update requires a dashboard restart for absolute-heading alignment')
        if not hasattr(validated, 'avoidance'):
            raise ValueError('Obstacle-checked velocity tracking requires a dashboard restart')
        self.avoidance = copy.deepcopy(validated.avoidance)
        self.display = copy.deepcopy(validated.display)
        self.hold_heading = False   # True: keep the heading measured at start, never yaw
        # True for an in-flight nudge: never freeze to turn; turn while moving.
        self.continuous = False
        self.min_pace = .6
        self._fixed_yaw = None
        self.source = validated.source
        self.obstacle_guard = validated.obstacle_guard
        self.points = [tuple(point) for point in validated.points]
        self.times = list(validated.times)
        self.anchor = copy.deepcopy(validated.anchor)
        self.reference = validated.reference
        self.radius = float(validated.radius)
        self.xy_speed = min(1., float(validated.xy_speed))
        self.z_speed = min(.3, float(validated.z_speed))
        self.floor, self.ceiling = float(validated.floor), float(validated.ceiling)
        if (not all(math.isfinite(v) for v in (self.radius, self.xy_speed, self.z_speed,
                                              self.floor, self.ceiling))
                or self.radius <= 0 or self.xy_speed <= 0 or self.z_speed <= 0
                or self.floor >= self.ceiling):
            raise ValueError('alternate follower requires valid course limits')
        self.traj = self._trajectory(self.points)
        params = TrackerParams(ff_mode='inverse', tau_v=.55, t_delay=.10,
                               v_max=self.xy_speed, z_floor=self.floor, z_ceiling=self.ceiling,
                               guard_margin=0., yaw_mode='path', yaw_rate_max=MAX_YAW_RATE)
        self.tracker = Tracker(self.traj, params)
        self._init_state(float(self._first_heading))

    def set_points(self, points):
        """Pass-through moved the route (same samples, same times): rebuild the trajectory the
        tracker follows, keeping its progress along it."""
        traj = self._trajectory([tuple(p) for p in points])
        self.points = [tuple(p) for p in points]
        self.traj = self.tracker.traj = traj

    def _trajectory(self, pts):
        points, times = np.asarray(pts, float), np.asarray(self.times, float)
        if (points.shape != (len(times), 3) or len(times) < 2 or not np.isfinite(points).all()
                or not np.isfinite(times).all() or np.any(np.diff(times) <= 0)):
            raise ValueError('alternate follower needs finite XYZ and increasing retimed samples')
        try:
            with np.errstate(over='raise', divide='raise', invalid='raise'):
                velocity = np.gradient(points, times, axis=0, edge_order=2 if len(times) > 2 else 1)
                acceleration = np.gradient(velocity, times, axis=0, edge_order=2 if len(times) > 2 else 1)
                headings = np.asarray(tangent_yaws(points, initial_yaw=float(self.anchor['yaw_rad'])))
                heading_rate = np.gradient(headings, times, edge_order=2 if len(times) > 2 else 1)
        except (FloatingPointError, OverflowError) as exc:
            raise ValueError('retimed route derivatives cannot be represented') from exc
        velocity[[0, -1]] = 0.
        acceleration[[0, -1]] = 0.
        heading_rate[[0, -1]] = 0.
        if not all(np.isfinite(values).all() for values in (velocity, acceleration, headings, heading_rate)):
            raise ValueError('non-finite route derivatives')
        self._first_heading = float(headings[0])
        return Trajectory(times, points, velocity, acceleration, headings, heading_rate)

    def _init_state(self, first_heading):
        headings = [first_heading]
        self.active, self.state = True, 'running'
        self.reason = 'Aligning with the first course tangent'
        self.phase = 'initial_align'
        self._initial_aligned = False
        self._alignment_s = 0.
        self._pace = 1.
        self._desired_heading = float(headings[0])
        self.yaw_rate = 0.
        self.last_velocity = [0., 0., 0.]
        self.position_hold = None
        self.commanded_heading = None
        self._alignment_turn = None
        self._alignment_elapsed = 0.
        self._previous_yaw = None
        self._measured_yaw_rate = None
        self._heading_error = 0.
        self._heading_blocked = False
        self._heading_wait_s = 0.
        self._heading_settle_s = 0.

    @property
    def elapsed(self):
        return float(self.tracker.tau)

    @property
    def reference_position(self):
        return [float(value) for value in self.traj.position(self.elapsed)]

    @property
    def desired_heading(self):
        return self._desired_heading

    def sample(self, t):
        return [float(value) for value in self.traj.position(t)]

    def _yaw_output(self, target_rate, dt):
        target_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, float(target_rate)))
        change = max(-MAX_YAW_ACCEL * dt, min(MAX_YAW_ACCEL * dt, target_rate - self.yaw_rate))
        self.yaw_rate += change
        if abs(self.yaw_rate) < 1e-12:
            self.yaw_rate = 0.
        return self.yaw_rate

    def _guard_velocity(self, velocity, position, dt):
        velocity = np.asarray(velocity, float).copy()
        velocity[2] = np.clip(velocity[2], -self.z_speed, self.z_speed)
        norm = float(np.linalg.norm(velocity))
        if norm > self.xy_speed:
            velocity *= self.xy_speed / norm
        endpoint = position + velocity * dt
        center = np.asarray(self.anchor['origin_enu'][:2], float)
        radial = endpoint[:2] - center
        distance = float(np.linalg.norm(radial))
        if distance > self.radius:
            endpoint[:2] = center + radial * (self.radius / distance)
        endpoint[2] = np.clip(endpoint[2], self.floor, self.ceiling)
        velocity = (endpoint - position) / dt
        velocity[2] = np.clip(velocity[2], -self.z_speed, self.z_speed)
        norm = float(np.linalg.norm(velocity))
        if norm > self.xy_speed:
            velocity *= self.xy_speed / norm
        return velocity

    def _snapshot_tracker(self):
        return (self.tracker.tau, self.tracker.taudot, self.tracker.integral.copy(),
                self.tracker.fixed_yaw, copy.deepcopy(self.tracker.last))

    def _restore_tracker(self, snapshot):
        (self.tracker.tau, self.tracker.taudot, self.tracker.integral,
         self.tracker.fixed_yaw, self.tracker.last) = snapshot

    def _velocity_output(self, target, position, dt):
        """Bound ordinary command acceleration; envelope guards take priority."""
        previous = np.asarray(self.last_velocity, float)
        delta = np.asarray(target, float) - previous
        delta[2] = np.clip(delta[2], -MAX_VERTICAL_ACCEL * dt, MAX_VERTICAL_ACCEL * dt)
        magnitude = float(np.linalg.norm(delta))
        if magnitude > MAX_LINEAR_ACCEL * dt:
            delta *= MAX_LINEAR_ACCEL * dt / magnitude
        output = self._guard_velocity(previous + delta, position, dt)
        self.last_velocity = [float(value) for value in output]
        return list(self.last_velocity)

    def _align_initial(self, dt, position, yaw, speed):
        if self._alignment_turn is None:
            self.position_hold = [float(value) for value in position]
            self._alignment_turn = YawTracker(yaw, max_rate=ALIGN_YAW_RATE,
                                             max_accel=ALIGN_YAW_ACCEL)
        self._alignment_elapsed += dt
        self.commanded_heading = self._alignment_turn.step(self._desired_heading, dt)
        self.yaw_rate = 0.  # Absolute yaw is carried by the pose, never a simultaneous twist.
        self.last_velocity = [0., 0., 0.]
        settled = (abs(self._heading_error) <= ENTER_HEADING and speed <= .15
                   and math.dist(position, self.position_hold) <= .15
                   and self._measured_yaw_rate is not None
                   and abs(self._measured_yaw_rate) <= SETTLE_YAW_RATE
                   and abs(self._alignment_turn.rate) <= 1e-6
                   and abs(wrap_angle(self._desired_heading-self.commanded_heading)) <= 1e-6)
        self._alignment_s = self._alignment_s + dt if settled else 0.
        if self._alignment_s + 1e-9 >= .2:
            self._initial_aligned = True
            self.phase, self.reason = 'following', 'Initial heading settled; starting velocity tracking'
        elif self._alignment_elapsed >= ALIGN_TIMEOUT_S:
            self.stop('aborted', 'Initial heading did not settle within 12 s; holding')
        return list(self.last_velocity), 0.

    def step(self, dt, position, yaw, measuredspeed):
        """Return bounded world-ENU (velocityXYZ, yaw_rate), without publishing.

        During initial alignment position_hold/commanded_heading request one
        pose setpoint instead. Course yaw follows the path, independent of the
        inverse-model velocity command. Heading waits freeze path progress and
        ramp translation down. Completion requests the caller's actual-position hold.
        """
        if not self.active:
            return [0., 0., 0.], 0.
        try:
            position = np.asarray(position, float)
            valid = (position.shape == (3,) and np.isfinite(position).all()
                     and math.isfinite(yaw) and math.isfinite(dt))
        except (TypeError, ValueError):
            valid = False
        if not valid:
            self.stop(reason='Invalid alternate follower input')
            return [0., 0., 0.], 0.
        dt = max(0., min(float(dt), .2))
        if dt == 0:
            return list(self.last_velocity), self.yaw_rate
        self._measured_yaw_rate = (None if self._previous_yaw is None else
                                  wrap_angle(yaw-self._previous_yaw) / dt)
        self._previous_yaw = float(yaw)
        if self.hold_heading:
            if self._fixed_yaw is None:
                self._fixed_yaw = float(yaw)
            self._desired_heading = self._fixed_yaw
        else:
            self._desired_heading = float(self.traj.sample(self.elapsed)[3])
        self._heading_error = wrap_angle(self._desired_heading-yaw)
        try:
            valid_speed = (measuredspeed is not None and math.isfinite(measuredspeed) and measuredspeed >= 0)
        except (TypeError, ValueError):
            valid_speed = False
        if not valid_speed:
            self.reason = 'Waiting for horizontal velocity'
            self.last_velocity = [0., 0., 0.]
            return list(self.last_velocity), self._yaw_output(0., dt)
        if not self._initial_aligned:
            return self._align_initial(dt, position, yaw, measuredspeed)

        self.position_hold = None
        self.commanded_heading = None
        error = self._heading_error
        if abs(error) >= EXIT_HEADING and not self.continuous:
            self._heading_blocked = True
        if self._heading_blocked:
            # Keep a fixed geometric heading while the path clock is paused.
            # A braking/correction vector must not retarget this turn.
            self._heading_wait_s += dt
            settled = (abs(error) <= ENTER_HEADING and measuredspeed <= .15
                       and self._measured_yaw_rate is not None
                       and abs(self._measured_yaw_rate) <= SETTLE_YAW_RATE)
            self._heading_settle_s = self._heading_settle_s + dt if settled else 0.
            self.tracker.taudot = 0.
            self.phase, self.reason = 'heading_wait', 'Aligning with the fixed course tangent'
            velocity = self._velocity_output(np.zeros(3), position, dt)
            rate = self._yaw_output(self.tracker.p.k_yaw * error, dt)
            if self._heading_settle_s >= .2 and abs(rate) <= SETTLE_YAW_RATE:
                self._heading_blocked = False
                self._heading_wait_s = self._heading_settle_s = 0.
            elif self._heading_wait_s >= ALIGN_TIMEOUT_S:
                self.stop('aborted', 'Course heading did not settle within 12 s; holding')
                return list(self.last_velocity), self.yaw_rate
            return velocity, rate

        u = max(0., min(1., (abs(error) - ENTER_HEADING) / (EXIT_HEADING - ENTER_HEADING)))
        self._pace = 1. - u * u * (3. - 2. * u)
        if self.continuous:
            self._pace = max(self._pace, self.min_pace)

        snapshot = self._snapshot_tracker()
        # hole_scale < 1 near a marked hole (dash_core): thread it slowly, this follower lags bends.
        velocity, _ = self.tracker.step(dt, position, yaw,
                                        speed_scale=self._pace*getattr(self, 'hole_scale', 1.))
        velocity = self._guard_velocity(velocity, position, dt)
        if not np.isfinite(velocity).all():
            self._restore_tracker(snapshot)
            self.stop(reason='Alternate follower produced invalid velocity')
            return [0., 0., 0.], 0.
        if self.tracker.finished and end_reached(self, position) and measuredspeed <= .15:
            self.phase, self.reason = 'finishing', 'Settling at the final course point'
            velocity = self._velocity_output(np.zeros(3), position, dt)
            rate = self._yaw_output(0., dt)
            if rate == 0. and np.linalg.norm(velocity) <= 1e-6:
                self.stop('complete', 'Course complete; holding at the final point')
            return list(self.last_velocity), rate

        self.phase = 'following'
        self.reason = ('Slowing to keep the camera facing travel' if self._pace < .99 else
                       'Following with the alternate velocity tracker')
        # Feedforward and yaw feedback refer to the same geometric heading.
        yaw_ff = 0. if self.hold_heading else float(self.traj.sample(snapshot[0])[4]) * self.tracker.taudot
        rate = self._yaw_output(yaw_ff + self.tracker.p.k_yaw * error, dt)
        return self._velocity_output(velocity, position, dt), rate

    def stop(self, state='stopped', reason='Course stopped; holding'):
        self.active, self.state, self.reason = False, state, reason
        self.phase = state
        self.yaw_rate = 0.
        self.last_velocity = [0., 0., 0.]

    def status(self):
        return dict(active=self.active, state=self.state, reason=self.reason,
                    controller=self.controller, phase=self.phase,
                    progress=round(self.elapsed / self.traj.duration, 4),
                    path_time_s=round(self.elapsed, 2), duration_s=round(self.traj.duration, 2),
                    radius_m=self.radius, xy_speed_mps=self.xy_speed,
                    route_source=self.source, obstacle_guard=self.avoidance is not None,
                    yaw_rate_dps=round(math.degrees(self.yaw_rate), 2),
                    heading_deg=round(math.degrees(self._desired_heading), 2),
                    heading_policy='hold_start' if self.hold_heading else 'course_tangent',
                    heading_error_deg=round(math.degrees(self._heading_error), 2),
                    measured_yaw_rate_dps=(None if self._measured_yaw_rate is None else
                                           round(math.degrees(self._measured_yaw_rate), 2)),
                    alignment_elapsed_s=round(self._alignment_elapsed, 2),
                    heading_wait_s=round(self._heading_wait_s, 2),
                    command_mode='pose' if self.position_hold is not None or not self.active else 'twist',
                    clock_rate=round(self.tracker.taudot, 3),
                    model=dict(tau_v_s=.55, delay_s=.10, measured=False))

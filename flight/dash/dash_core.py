"""Dashboard control core for skybeta-12. PURE Python (no ROS), so the safety rules are testable.

Control limits (the browser is not an independent safety link):
  * The dashboard must send a heartbeat (default every 100 ms). The drone side only publishes
    setpoints while the OWNER's heartbeat is younger than hb_timeout_s. If it goes quiet, the
    node stops publishing and latches FAILSAFE. The flight interface keeps holding the last
    position; it does not switch flight modes. RC takeover requires the pilot's mode switch.
  * Movement uses /skybeta/setpoint/{pose,twist}. The node can send a normal arm/disarm
    request after this core validates ownership. PX4 decides whether to accept it.
    It never forces arming or changes modes.
  * Pose setpoints are NOT speed-limited by the interface and there is no horizontal fence, so
    this core ramps every setpoint (v_xy, v_z) and keeps the target inside a fence around the
    point where control was engaged. Leaving the fence (actual position) trips FAILSAFE.
  * If the pilot takes over (Offboard ends) the node stops and stays stopped (mode PILOT).

Coordinates: ENU in PX4's local frame (flight_interface maps ENU -> NED by REP-103 with no
correction, so setpoints are effectively PX4 local positions). Heights are relative to the GROUND
where control was engaged (PX4 local z at engage, vehicle landed). The interface is configured
with use_px4_height so its altitude limits also use the landed PX4 height. Engage is refused
if that ground reference disagrees with the current PX4 position.
"""
from __future__ import annotations

import math
import os
import numpy as np
import re
from dataclasses import dataclass, field
from heading_control import FacingTravel, limited_horizontal_step
from course_follow import CourseRun
from course_safety import observations, ObstacleGuard, MAX_COURSE_SPEED_MPS, route_ahead

# After an obstacle stop: smooth_position resumes at full course speed; the
# velocity tracker (noisy-position feedback) came within 0.18 m of the wall at
# 1 m/s in jittered simulation, so its remaining course is capped.
VELOCITY_TRACKER_DETOUR_MPS = .35
# Continuous avoidance: look this far along the planned route (beyond the
# stopping distance) and nudge the route, without stopping, if it comes within
# NUDGE_TRIGGER_M of a wall. Nudged routes keep the planned 0.40 m, so a fresh
# nudge does not immediately retrigger. One planning attempt per NUDGE_RETRY_S.
NUDGE_LOOKAHEAD_M = 2.0
EGO_CARROT_M = 2.0       # EGO mode: goal this far ahead along the route...
EGO_GOAL_STEP_M = .3     # ...re-sent when it moves this much
EGO_STALE_S = .3         # no EGO setpoint this long: hold position
REJOIN_AHEAD_M = .8      # rejoin the planned route this far ahead (a glide back, not a side step)
REJOIN_CHECK_M = 3.0     # the planned route must be clear this far ahead of the rejoin point
REJOIN_AFTER_S = .8      # no rejoin attempt this soon after a nudge / replan (one clean 2 Hz depth frame later)
REJOIN_VIEW_DEG = 35.     # rejoin only if the nudge's trigger is inside the camera view (half-angle)...
REJOIN_VIEW_M = (.4, 3.5) # ...and this far ahead, and the latest frame shows nothing there
REJOIN_CLEAR_M = .40     # hysteresis: NUDGE_TRIGGER_M (0.35) + 0.05, clearer than what triggers a nudge
NUDGE_TRIGGER_M = .35
NUDGE_RETRY_S = 1.0
NUDGE_MAX_AGE_S = 1.5
RESUME_SPEED_MPS = .25
RESUME_START_M = .40
RESUME_SETTLE_S = .3

IDLE, ENGAGED, LANDING, FAILSAFE, PILOT = "idle", "engaged", "landing", "failsafe", "pilot"


def _live(depth):
    try:
        observations(depth)
        return True
    except (ValueError, TypeError):
        return False
HOLE_SLOW_M = 2.5      # velocity follower: within this of a marked hole (covers the 1 m blend-in)...
HOLE_SLOW_SCALE = .35  # ...its course clock runs at this fraction (~0.35 m/s at 1 m/s)


@dataclass
class DashParams:
    hb_timeout_s: float = 1.0          # owner heartbeat older than this -> FAILSAFE
    owner_stale_s: float = 3.0         # another client may claim control after this long
    pos_timeout_s: float = 0.5         # PX4 local position older than this -> FAILSAFE
    fc_timeout_s: float = 1.0          # /skybeta/state older than this -> FAILSAFE
    fence_r_m: float = 1.5             # target stays within this of the engage point
    fence_margin_m: float = 0.5        # actual position beyond fence_r + margin -> FAILSAFE
    min_alt_m: float = 0.3             # flight_interface floor (min_altitude_m)
    max_alt_m: float = 2.0             # startup supplies the shared altitude limits
    default_alt_m: float = 0.5
    v_xy: float = 1.0                  # horizontal setpoint ramp, m/s
    course_v_xy: float = 1.0           # course-only horizontal target ramp, m/s
    v_z: float = 0.3
    nudge_xy_max_m: float = 0.5
    nudge_z_max_m: float = 0.3
    land_v: float = 0.2                # final descent twist, m/s down
    land_timeout_s: float = 10.0       # touchdown phase longer than this -> release to pilot
    floor_tol_m: float = 0.2           # interface floor vs ground + min_alt: refuse engage beyond this
    land_after_course: bool = True    # a completed course lands like the Land button
    # Operator choice (RC kill + dashboard kill are the safety): once flying, nothing on the laptop
    # side and no dashboard judgement holds the flight. Only lost PX4 position/link, RC takeover,
    # disarm, a kill, or a follower crash stop it. False restores the old holds (heartbeat, fence,
    # tracking error, landed-mid-course, takeoff timeout, in-flight readiness, reference changes).
    no_holds: bool = field(default_factory=lambda: os.environ.get('DASH_LEGACY_HOLDS') != '1')  # =1: old holds
    # Course + return hop (second Fly button): at the end of the course PX4 lands it (Land mode),
    # then the Fly button takes off again from there into the return hop.
    course_ceiling_m: float = 2.0      # our own ceiling for courses / manual heights (max_alt_m = interface limit)
    hop_ceiling_m: float = 2.5         # ceiling while the return hop flies (needs max_alt_m >= this)
    hop_end_dz_m: float = 0.4          # chained course ends on time + 15 cm horizontally, height within this
    hop_px4_land_wait_s: float = 3.0   # PX4 not in Land by then: the dashboard lands it itself
    hop_start_tol_m: float = 0.30      # hop takeoff: this close (horizontally) to the hop's first point
    course_v_z: float = 0.8            # course climb/descent rate (manual moves keep v_z); pose has no interface speed cap
    heading_mode: str = 'path_tangent'  # 'hold_start': never yaw; 'path_tangent': face travel
    obstacle_mode: str = 'avoid'       # 'shadow': depth is display-only; fly the original course
    # True: the old rule (start only on a fresh obstacle-checked route with live depth). False (operator
    # choice): fly the planned route when no fresh check exists; obstacles are handled in flight.
    require_checked_route: bool = False

    def __post_init__(self):
        if self.heading_mode not in ('hold_start', 'path_tangent'):
            raise ValueError('heading_mode must be hold_start or path_tangent')
        if self.obstacle_mode not in ('avoid', 'shadow', 'ego'):
            raise ValueError('obstacle mode must be avoid or shadow')
        if isinstance(self.v_xy, bool) or not math.isfinite(self.v_xy) or self.v_xy <= 0:
            raise ValueError("horizontal speed must be finite and positive")
        if (isinstance(self.course_v_xy, bool) or not math.isfinite(self.course_v_xy)
                or self.course_v_xy <= 0):
            raise ValueError("course speed must be finite and positive")
        self.v_xy = min(self.v_xy, MAX_COURSE_SPEED_MPS)
        self.course_v_xy = min(self.course_v_xy, MAX_COURSE_SPEED_MPS)


@dataclass
class Pos:                             # PX4 local position, ENU
    t: float
    x: float
    y: float
    z: float
    yaw: float                         # ENU yaw, rad
    valid: bool = True
    vx: float | None = None            # ENU, fresh horizontal velocity or unknown
    vy: float | None = None
    vz: float | None = None


@dataclass
class Fc:                              # from /skybeta/state
    t: float
    px4_link: bool
    armed: bool
    offboard_active: bool
    landed: bool
    nav_state_name: str = ""
    origin_z: float = 0.0


@dataclass
class DashCore:
    p: DashParams = field(default_factory=DashParams)
    mode: str = IDLE
    reason: str = ""
    owner: str | None = None
    owner_hb: float = -1e9
    engage_xy: tuple = (0.0, 0.0)
    engage_yaw: float = 0.0
    origin_z: float = 0.0
    ground_z: float = 0.0
    home: tuple | None = None
    ego_goal: list | None = None        # EGO mode: current carrot goal (the node publishes it)
    ego_goal_seq: int = 0
    ego_setpoint: tuple | None = None   # EGO mode: (monotonic receive time, [x, y, z], [vx, vy, vz])
    target: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    cmd: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    was_offboard: bool = False
    land_phase: str = ""
    land_t0: float = 0.0
    last_step: float | None = None
    events: list = field(default_factory=list)
    last_out: tuple | None = None
    heading: FacingTravel = field(default_factory=FacingTravel)
    course_run: CourseRun | None = None
    course_radius: float | None = None
    course_controller: str = 'smooth_position'
    course_yaw_rate: float = 0.
    queued_course: CourseRun | None = None
    deferred_course: dict | None = None
    paused_course: dict | None = None
    hop: dict | None = None             # return hop: {state: armed|land|ready|flying, path, limit, t0}
    px4_land_request: float | None = None   # the node sends PX4 Land mode once for this request time
    nudge_request: dict | None = None   # obstacle ahead on the route; planning a detour in flight
    nudge_retry_at: float = 0.
    replan_reason: str = ''
    launch_state: str = 'idle'
    launch_reason: str = ''
    launch_armed_at: float | None = None
    launch_settle_at: float | None = None

    def select_course_controller(self, now, controller, pos, fc):
        if controller not in ('smooth_position','velocity_tracker'):
            return False, 'unknown course follower'
        if self.paused_course is not None or self.deferred_course is not None or self.queued_course is not None or (self.course_run is not None and self.course_run.active):
            return False, 'stop the course before changing follower'
        if (pos is None or not pos.valid or now-pos.t>self.p.pos_timeout_s
                or fc is None or not fc.px4_link or now-fc.t>self.p.fc_timeout_s):
            return False, 'fresh position and flight status are required'
        if fc.armed or not fc.landed:
            return False, 'change follower only while disarmed and landed'
        self.course_controller=controller
        self.log(now,'Course follower selected: '+controller)
        return True, 'Course follower selected'

    def select_course_options(self, now, pos, fc, obstacle_mode=None, heading_mode=None):
        """Set collision avoidance (avoid/shadow) and heading (hold_start/path_tangent)
        from the dashboard; same grounded/no-course rules as the follower choice."""
        if obstacle_mode is not None and obstacle_mode not in ('avoid', 'shadow', 'ego'):
            return False, 'collision avoidance must be avoid, shadow or ego'
        if heading_mode is not None and heading_mode not in ('hold_start', 'path_tangent'):
            return False, 'heading must be hold_start or path_tangent'
        if obstacle_mode is None and heading_mode is None:
            return False, 'nothing to change'
        if self.paused_course is not None or self.deferred_course is not None or self.queued_course is not None or (self.course_run is not None and self.course_run.active):
            return False, 'stop the course before changing course options'
        if (pos is None or not pos.valid or now-pos.t>self.p.pos_timeout_s
                or fc is None or not fc.px4_link or now-fc.t>self.p.fc_timeout_s):
            return False, 'fresh position and flight status are required'
        if fc.armed or not fc.landed:
            return False, 'change course options only while disarmed and landed'
        if obstacle_mode is not None:
            self.p.obstacle_mode = obstacle_mode
            self.log(now, 'Collision avoidance ' + {'avoid': 'ON', 'ego': 'EGO planner (local replanning by EGO)'}.get(
                obstacle_mode, 'OFF (depth display only)'))
        if heading_mode is not None:
            self.p.heading_mode = heading_mode
            self.heading.fixed = heading_mode == 'hold_start'
            self.log(now, 'Heading: ' + ('hold start heading' if heading_mode == 'hold_start' else 'face travel (path tangent)'))
        return True, 'Course options saved'

    def _tracking_error(self, run, pos):
        here = (pos.x, pos.y, pos.z)
        if run.controller != 'velocity_tracker':
            return math.dist(self.cmd, here)
        # Distance to the planned route near the reference (1 s behind to 1.5 s
        # ahead): cross-track drift still trips, along-track lag during a
        # nudge handover or a speed change does not.
        best = math.dist(run.reference_position, here)
        t, end = max(0., run.elapsed-1.), min(run.times[-1], run.elapsed+1.5)
        while t <= end:
            best = min(best, math.dist(run.sample(t), here))
            t += .1
        return best

    def apply_nudge(self, now, path, velocity_limit_mps=1.):
        """Swap in a nudged remainder while flying: no stop, no settle, no realign."""
        request, run = self.nudge_request, self.course_run
        self.nudge_request = None
        if (request is None or run is None or request['run'] is not run or not run.active
                or self.mode != ENGAGED or now-request.get('planned_at', -1e9) > NUDGE_MAX_AGE_S):
            return False, 'nudge no longer applies'
        if getattr(getattr(run, 'pass_through', None), 'state', None) in ('hover', 'commit'):
            return False, 'hole control in progress'
        try:
            if path.get('reference') != run.reference:
                raise ValueError('course reference changed')
            points = [list(p) for p in path['points']]
            # The aircraft kept flying while the nudge was planned: join the new
            # route at its closest point to the current command, never behind it.
            window = points[:min(len(points), 400)]
            i = min(range(len(window)), key=lambda k: math.dist(window[k][:2], self.cmd[:2]))
            points = [list(self.cmd)] + points[i+1:]
            if len(points) < 2:
                raise ValueError('nudged route has no remainder')
            old_guard = run.obstacle_guard
            if old_guard is not None and path.get('avoidance'):
                # Only confirmed cells (seen in 2 frames) carry over; the planner's wall list also has the
                # current frame's unconfirmed ones, and a one-frame speckle must not be remembered forever.
                path = dict(path, avoidance=dict(path['avoidance'], walls=list(old_guard.cells.values()),
                                                 cell_m=old_guard.cell_m))
            cap = VELOCITY_TRACKER_DETOUR_MPS if self.course_controller == 'velocity_tracker' else None
            run_new = self._make_course(dict(path, points=points, times_s=None), run.radius, velocity_limit_mps,
                                        xy_cap=cap)
            if old_guard is not None and run_new.obstacle_guard is not None:
                run_new.obstacle_guard.inherit(old_guard)
            rejoin = bool(request.get('rejoin'))
            self._inherit_plan(run_new, run, now, nudged=not rejoin)
            run_new.detour_trigger = None if rejoin else (request.get('trigger') or None)
            run_new.detour_version = request.get('trigger_version')
            if rejoin:
                self.nudge_retry_at = now+NUDGE_RETRY_S     # no instant re-nudge after a glide back
            if hasattr(run_new, '_initial_aligned'):   # velocity tracker: keep flying
                run_new._initial_aligned = True
                run_new.continuous = True
                run_new.phase = 'following'
                run_new.last_velocity = list(getattr(run, 'last_velocity', [0., 0., 0.]))
                if getattr(run, '_fixed_yaw', None) is not None:
                    run_new._fixed_yaw = run._fixed_yaw
        except (ValueError, TypeError, KeyError, OverflowError, ImportError) as exc:
            self.nudge_retry_at = now+NUDGE_RETRY_S
            return False, str(exc)
        self.course_run = run_new
        if rejoin:
            self.replan_reason = 'Planned route clear again; back on it'
            self.log(now, 'Planned route clear again: gliding back onto it')
        else:
            self.replan_reason = 'Nudged around an obstacle; continuing'
            self.log(now, 'Obstacle ahead: route nudged in flight, no stop')
        return True, self.replan_reason

    def _ego_target(self, now, run, pos):
        """EGO-Planner mode: the goal is a carrot EGO_CARROT_M ahead along the route (published by the
        node); the target is EGO's latest setpoint, or a hold when EGO is silent. Fence, floor, ceiling,
        heartbeat and stop still apply to the command."""
        pts = run.points
        lo = getattr(run, 'ego_i', 0)
        hi = min(len(pts), lo+400)
        i = lo+min(range(hi-lo), key=lambda k: math.dist(pts[lo+k][:2], (pos.x, pos.y)))
        run.ego_i = max(lo, i)
        j, acc = run.ego_i, 0.
        while j+1 < len(pts) and acc < EGO_CARROT_M:
            acc += math.dist(pts[j][:2], pts[j+1][:2])
            j += 1
        goal = [float(v) for v in pts[j]]
        if self.ego_goal is None or math.dist(goal, self.ego_goal) > EGO_GOAL_STEP_M or (
                j == len(pts)-1 and self.ego_goal != goal):
            self.ego_goal = goal
            self.ego_goal_seq += 1
        sp = self.ego_setpoint
        if sp is None or now-sp[0] > EGO_STALE_S:
            self.target = list(self.cmd)
            run.reason = 'EGO planner: waiting for a trajectory (holding)'
        else:
            self.target = [float(v) for v in sp[1]]
            run.reason = 'EGO planner: following its trajectory'
        if j == len(pts)-1 and math.dist((pos.x, pos.y, pos.z), pts[-1]) <= .15:
            run.stop('complete', 'Course complete (EGO planner)')
            self.target = list(self.cmd)
            self.heading.hold()
            self.log(now, run.reason)
            self._land_if_complete(now, run)

    def _pass_through(self, now, run, pos, depth, dt):
        """Opening on the route ahead (tunnel, window): keep that stretch of the route
        centred in it, sideways and up/down, re-measured on every depth frame."""
        if run.avoidance is None or self.p.obstacle_mode in ('shadow', 'ego'):
            return
        from pass_through import PassThrough
        pt = getattr(run, 'pass_through', None)
        if pt is None:
            # hole control: only holes marked inside an avoidance zone the operator drew
            holes = ((getattr(run, 'marks', None) or {}).get('control_holes')) or ()
            pt = run.pass_through = PassThrough(run.points, holes=holes, done=getattr(run, 'holes_done', ()))
        live = isinstance(depth, dict) and depth.get('status') == 'live'
        cells = depth.get('cells3d') if live else None
        frame = (depth.get('geometry_version') or depth.get('stamp_ns') or depth.get('produced_wall_ns')) if live else None
        was = pt.state
        try:
            route = pt.update((pos.x, pos.y, pos.z), cells, frame, dt,
                              floor=self._floor(), ceiling=self._ceiling())
        except (ValueError, TypeError, IndexError):
            return
        from pass_through import COMMIT_PACE
        near = any(math.hypot(h['at'][0]-pos.x, h['at'][1]-pos.y) <= HOLE_SLOW_M
                   for h in pt.holes if not h['done'])
        slow = 0. if pt.hold else COMMIT_PACE if pt.state == 'commit' else HOLE_SLOW_SCALE if near else 1.
        if run.controller == 'velocity_tracker':
            if pt.state == 'commit':
                slow = min(slow, .25)      # it tracks the blind line through the gap more loosely: slower
            run.hole_scale = slow          # 0 while hovering: its course clock stops
        else:
            run.hold_clock, run.pace_scale = pt.hold, (slow if slow > 0 else 1.)
        if pt.state in ('too_close', 'gave_up'):
            self.stop_course(now, 'Hole control: '+pt.reason)    # explicit stop, never a silent hover
            return
        if route is not None:
            ex, ey = self.engage_xy
            if any(math.hypot(p[0]-ex, p[1]-ey) > run.radius for p in route):
                self.stop_course(now, 'Hole control: the line through the hole leaves the course radius')
                return
            try:
                if hasattr(run, 'set_points'):
                    run.set_points(route.tolist())      # velocity tracker: rebuild its trajectory
                else:
                    run.points = [tuple(p) for p in route.tolist()]
            except (ValueError, TypeError, FloatingPointError) as exc:
                self.stop_course(now, 'Hole control: could not install the line through the hole ('+str(exc)[:60]+')')
                return
        if pt.state != was:
            self.log(now, {'hover': 'Hole ahead: hovering to measure it before committing',
                           'commit': 'Hole locked in the world frame: lined up, flying straight through',
                           'normal': 'Through the hole; back on the planned route'}[pt.state])

    def fence_radius(self):
        return self.p.fence_r_m if self.course_radius is None else self.course_radius

    def stop_course(self, now, reason='Course stopped; holding', hold=True):
        self.nudge_request = None
        if self.hop is not None:
            self.log(now, 'Return hop cancelled')
            self.hop = None
        if self.paused_course is not None:
            self.paused_course = None
            self.replan_reason = reason
            if hold:
                self.target = list(self.cmd)
                self.heading.hold()
            self.log(now, reason)
        if self.deferred_course is not None or self.queued_course is not None:
            self.deferred_course = None
            self.queued_course = None
            self.launch_state, self.launch_reason = 'cancelled', reason
            self.launch_armed_at = self.launch_settle_at = None
            if hold:
                self.target = list(self.cmd)
                self.heading.hold()
            self.log(now, reason)
        if self.course_run is not None and self.course_run.active:
            self.course_run.stop(reason=reason)
            if hold:
                self.target = list(self.cmd)
                self.heading.hold()
            self.log(now, reason)
        self.course_yaw_rate=0.

    def _make_course(self, path, radius, velocity_limit_mps=1., floor=None, ceiling=None, xy_cap=None):
        if self.p.obstacle_mode in ('shadow', 'ego'):
            if path.get('source', 'original') != 'original':
                raise ValueError('depth display-only mode requires the original course')
            path = dict(path, avoidance=None, source='original')
        anchor = path['anchor']
        origin = anchor['origin_enu']
        if (self.home is None or math.dist(self.home[:3], origin) > .02
                or abs(self.home[3]-anchor['yaw_rad']) > .001):
            raise ValueError('home and course reference disagree; reset on the launch mark')
        xy_speed = self.p.course_v_xy
        if self.course_controller == 'velocity_tracker':
            if (type(velocity_limit_mps) not in (int, float) or
                    not math.isfinite(velocity_limit_mps) or velocity_limit_mps <= 0):
                raise ValueError('invalid velocity interface limit')
            xy_speed = min(xy_speed, velocity_limit_mps, 1.)
        if xy_cap is not None:
            xy_speed = min(xy_speed, xy_cap)
        run = CourseRun(path, radius, xy_speed, self.p.course_v_z,
                        self._floor() if floor is None else floor,
                        self._ceiling() if ceiling is None else ceiling)
        if self.course_controller == 'velocity_tracker':
            from alternate_velocity import AlternateVelocityRun
            run = AlternateVelocityRun(run, position_hold_supported=True)
            run.hold_heading = self.p.heading_mode == 'hold_start'
        run.marks = path.get('marks')   # marked holes: pass-through centres there
        run.plan = dict(points=[tuple(p) for p in run.points], times=list(run.times))  # for rejoining
        run.nudged = False
        if self.hop is not None and self.hop['state'] in ('armed', 'flying'):
            run.end_dz_m = self.p.hop_end_dz_m   # do not wait on the altitude estimate to finish
        return run

    def arm_hop(self, path, velocity_limit_mps=1.):
        """Chain the return hop after the next course (second Fly button). Checked now, flown later."""
        pts = path.get('points') or []
        if len(pts) < 2:
            return False, 'No return hop loaded (Advanced: import return hop)'
        top = max(p[2] for p in pts) - self.ground_z
        if top > min(self.p.max_alt_m, self.p.hop_ceiling_m) + 1e-6:
            return False, (f'Return hop reaches {top:.2f} m but the allowed height is '
                           f'{min(self.p.max_alt_m, self.p.hop_ceiling_m):.2f} m (flight interface limit '
                           f'{self.p.max_alt_m:g} m, hop ceiling {self.p.hop_ceiling_m:g} m)')
        self.hop = dict(state='armed', path=path, limit=velocity_limit_mps, t0=None)
        self.log(0. if self.last_step is None else self.last_step,
                 'Return hop armed: after the course PX4 lands, then Take off · return hop')
        return True, 'Return hop armed'

    def prepare_hop_takeoff(self, now, path, radius, pos, fc, velocity_limit_mps=1.):
        """Second takeoff: from where PX4 landed after the course, into the return hop."""
        if self.hop is None or self.hop['state'] != 'ready':
            return False, 'No return hop waiting (fly the course with the course + return hop button first)'
        self.hop['state'] = 'flying'           # hop ceiling + hop start rules for this preparation
        ok, msg = self.prepare_course(now, path, radius, pos, fc, None, velocity_limit_mps, hop_start=True)
        if not ok:
            self.hop['state'] = 'ready'
        return ok, msg

    def _inherit_plan(self, new, old, now, nudged):
        """A nudged / replanned / rejoined run keeps the course's original plan and marked holes."""
        new.plan = getattr(old, 'plan', None) or new.plan
        new.marks = getattr(old, 'marks', None)
        pt = getattr(old, 'pass_through', None)       # holes already flown (or being flown) stay done
        new.holes_done = (pt.done_holes() if pt is not None else []) + list(getattr(old, 'holes_done', []))
        new.nudged = nudged
        new.no_rejoin = bool(getattr(old, 'no_rejoin', False))
        new.plan_index = getattr(old, 'plan_index', 0)
        if nudged:
            self.rejoin_at = now+REJOIN_AFTER_S

    def _camera_pose(self):
        """(x, y, yaw) the depth camera looks from (dry run: the real carried pose)."""
        pose = getattr(self, '_camera_pos', None) or getattr(self, '_last_pos', None)
        return None if pose is None else (pose.x, pose.y, pose.yaw, pose.z)

    def rejoin_job(self, now):
        """After a nudge or replan: if the planned route ahead is clear again, inputs to plan a glide
        back onto it (the slow check runs outside the flight lock). None when not applicable."""
        run = self.course_run
        if (run is None or not run.active or not getattr(run, 'nudged', False) or getattr(run, 'no_rejoin', False)
                or self.mode != ENGAGED
                or self.nudge_request is not None or self.paused_course is not None
                or now < getattr(self, 'rejoin_at', 0.) or run.obstacle_guard is None
                or getattr(getattr(run, 'pass_through', None), 'state', None) in ('hover', 'commit')):
            return None
        # Positive evidence only: the cells that caused the nudge must still be ahead, inside the
        # camera's view, and absent from the latest frame. Beside/behind (out of view) or unknown
        # (a stop + replan): keep the detour.
        trigger = getattr(run, 'detour_trigger', None)
        pose = getattr(self, '_camera_pos', None) or getattr(self, '_last_pos', None)   # dry run: real camera
        g = run.obstacle_guard
        last = g._last
        if not trigger or pose is None or last is None:
            return None
        # Evidence must be fresh and new: the last usable frame under 0.6 s old, not the frame that
        # triggered the nudge, and two such frames in a row showing the trigger gone.
        if g._last_t is None or now-g._last_t > .6 or g._last_version == getattr(run, 'detour_version', None):
            return None
        tr = np.asarray(trigger, dtype=float).reshape(-1, 2)
        dx, dy = tr[:, 0]-pose.x, tr[:, 1]-pose.y
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        dist = np.hypot(dx, dy)
        ang = np.degrees(np.abs(np.arctan2(-s*dx+c*dy, c*dx+s*dy)))
        if np.any((dist < REJOIN_VIEW_M[0]) | (dist > REJOIN_VIEW_M[1]) | (ang > REJOIN_VIEW_DEG)):
            return None              # out of view: no evidence either way
        w = np.asarray(last[0], dtype=float).reshape(-1, 2)
        if len(w) and float(np.min(np.hypot(w[None, :, 0]-tr[:, None, 0], w[None, :, 1]-tr[:, None, 1]))) <= .15:
            run.rejoin_clear = (g._last_version, 0)
            self.rejoin_at = now+.5
            return None              # still seen there
        ver, count = getattr(run, 'rejoin_clear', (None, 0))
        if ver != g._last_version:
            run.rejoin_clear = (g._last_version, count+1)
        if run.rejoin_clear[1] < 2:
            return None
        plan = run.plan
        pts, times = plan['points'], plan['times']
        lo = max(0, getattr(run, 'plan_index', 0)-20)
        hi = min(len(pts), getattr(run, 'plan_index', 0)+400)      # this lap only, never a later pass
        dist = [math.dist(p[:2], self.cmd[:2]) for p in pts[lo:hi]]
        i = lo + min(range(len(dist)), key=dist.__getitem__)
        run.plan_index = max(getattr(run, 'plan_index', 0), i)
        along, j = 0., i
        while j+1 < len(pts) and along < REJOIN_AHEAD_M:
            along += math.dist(pts[j][:2], pts[j+1][:2])
            j += 1
        if j >= len(pts)-1 or j == 0:
            return None                      # too close to the end: just finish the detour
        ahead, k, reach = [tuple(self.cmd[:2])], j, 0.
        while k < len(pts) and reach < REJOIN_CHECK_M:
            ahead.append(tuple(pts[k][:2]))
            reach += math.dist(pts[k-1][:2], pts[k][:2]) if k > j else 0.
            k += 1
        if run.obstacle_guard.route_conflict(ahead, REJOIN_CLEAR_M, height_route=pts):
            self.rejoin_at = now+1.
            return None
        g = run.obstacle_guard
        return dict(run=run, t=now, points=list(pts), times=list(times), elapsed=times[j-1], start=tuple(self.cmd),
                    anchor=run.anchor, reference=run.reference, radius=run.radius,
                    walls=g.known_walls(pts), cell=g.cell_m, clearance=g.clearance,
                    floor=self._floor(), ceiling=self._ceiling())

    def commit_rejoin(self, now, job, path):
        """Queue the checked glide back as a nudge request (applied by the flight tick)."""
        if (path is None or path.get('source') != 'unchanged' or self.course_run is not job['run']
                or self.nudge_request is not None):
            self.rejoin_at = now+1.
            return False
        self.nudge_request = dict(run=job['run'], at=job['t'], path=path, planned_at=job['t'], rejoin=True)
        return True

    def resume_course(self, now, path, pos, fc, depth, velocity_limit_mps=1.):
        """Resume only a settled obstacle pause with a newly guarded remainder."""
        paused = self.paused_course
        if paused is None or self.mode != ENGAGED:
            return False, 'No obstacle pause to resume'
        if (not self.link_alive(now) or pos is None or not pos.valid
                or not 0 <= now-pos.t <= self.p.pos_timeout_s
                or fc is None or not fc.px4_link or not 0 <= now-fc.t <= self.p.fc_timeout_s
                or not fc.armed or not fc.offboard_active or fc.landed):
            return False, 'Waiting for current airborne control'
        # Tolerances sized for indoor hover wobble (VIO/position noise of several cm).
        settled = (all(v is not None and math.isfinite(v) for v in (pos.vx,pos.vy,pos.vz))
                   and math.hypot(pos.vx,pos.vy) <= RESUME_SPEED_MPS and abs(pos.vz) <= .15
                   and math.dist(self.cmd,(pos.x,pos.y,pos.z)) <= RESUME_START_M)
        if not settled:
            paused['settled_at'] = None
            return False, 'Waiting for the aircraft to stop'
        if paused.get('settled_at') is None:
            paused['settled_at'] = now
        if now-paused['settled_at'] < RESUME_SETTLE_S:
            return False, 'Settling before the new route'
        try:
            if path.get('reference') != paused['reference'] or self.course_controller != paused['controller']:
                raise ValueError('Course reference or follower changed')
            if not path.get('avoidance'):
                raise ValueError('Replacement route requires obstacle checks')
            if math.dist(path['points'][0],(pos.x,pos.y,pos.z)) > RESUME_START_M:
                raise ValueError('Aircraft moved; recompute the route from this hover')
            # Start the new route where the aircraft actually is; the guard
            # checks that first leg like any other command.
            here = [pos.x, pos.y, pos.z]
            path = dict(path, points=[here]+[list(p) for p in path['points'][1:]])
            self.cmd = list(here)
            guard = ObstacleGuard(path['avoidance'])
            guard.inherit(paused['guard'])
            guard.check(depth,(pos.x,pos.y,pos.z),(pos.vx,pos.vy),('pose',self.cmd),self.cmd,.1,now=now)
            path = dict(path, avoidance=dict(path['avoidance'],
                        walls=list(guard.cells.values()),cell_m=guard.cell_m))
            cap = VELOCITY_TRACKER_DETOUR_MPS if self.course_controller == 'velocity_tracker' else None
            run = self._make_course(path,paused['run'].radius,velocity_limit_mps,xy_cap=cap)
            self._inherit_plan(run, paused['run'], now, nudged=True)
            run.detour_trigger = None    # a guard stop + replan is never abandoned: stick to it
            run.no_rejoin = True
        except (ValueError,TypeError,KeyError,OverflowError,ImportError) as exc:
            return False,str(exc)
        self.course_run,self.paused_course = run,None
        self.course_radius = run.radius
        self.target = list(self.cmd)
        self.heading.hold()
        self.course_yaw_rate = 0.
        self.replan_reason = 'Following the checked detour'
        self.log(now,'Obstacle pause ended; following the checked remaining course')
        return True,self.replan_reason

    def prepare_course(self, now, path, radius, pos, fc, depth, velocity_limit_mps=1., hop_start=False):
        """Queue takeoff then course on the next manual arm; never arm or change mode."""
        if self.paused_course is not None or self.deferred_course is not None or self.queued_course is not None or (self.course_run and self.course_run.active):
            return False, 'course is already prepared or running'
        if (self.mode not in (IDLE, ENGAGED) or self.was_offboard) and not hop_start:
            return False, 'reset on the launch mark before preparing a course'
        if not self.link_alive(now) and not hop_start:
            return False, 'dashboard heartbeat lost'
        if (pos is None or not pos.valid or now-pos.t > self.p.pos_timeout_s
                or not all(math.isfinite(v) for v in (pos.x, pos.y, pos.z, pos.yaw))
                or fc is None or not fc.px4_link or now-fc.t > self.p.fc_timeout_s):
            return False, 'fresh position and flight status are required'
        if (fc.armed and not hop_start) or not fc.landed:
            return False, 'prepare automatic takeoff while disarmed and landed'
        if (any(v is None or not math.isfinite(v) for v in (pos.vx,pos.vy,pos.vz))
                or math.hypot(pos.vx,pos.vy) > .15 or abs(pos.vz) > .10):
            return False, 'fresh settled XYZ velocity is required before automatic takeoff'
        if abs(fc.origin_z-pos.z) > self.p.floor_tol_m:
            return False, 'flight interface and PX4 ground references disagree'
        try:
            if self.p.obstacle_mode not in ('shadow', 'ego') and self.p.require_checked_route:
                observations(depth)
            run = self._make_course(path, radius, velocity_limit_mps,
                floor=max(fc.origin_z, pos.z)+self.p.min_alt_m, ceiling=pos.z+self._top_m())
            if self.p.obstacle_mode not in ('shadow', 'ego') and run.avoidance is None:
                raise ValueError('automatic course requires an obstacle-checked path')
            if hop_start:
                gap = math.dist((pos.x, pos.y), run.points[0][:2])
                if gap > self.p.hop_start_tol_m:
                    raise ValueError(f'the drone is {gap:.2f} m from the return hop start '
                                     f'(max {self.p.hop_start_tol_m:g} m)')
            elif (math.dist((pos.x, pos.y), run.points[0][:2]) > .05
                    or math.dist((pos.x, pos.y, pos.z), self.home[:3]) > .10):
                raise ValueError('place the drone on the course launch mark before preparing')
            if run.obstacle_guard is not None and (_live(depth) or self.p.require_checked_route):
                run.obstacle_guard.check(depth, (pos.x,pos.y,pos.z), (pos.vx,pos.vy),
                                         ('pose', (pos.x,pos.y,run.points[0][2])),
                                         (pos.x,pos.y,pos.z), .1)
        except (ValueError, TypeError, KeyError, OverflowError, ImportError) as exc:
            return False, str(exc)
        self.origin_z, self.ground_z = fc.origin_z, pos.z
        self.engage_xy, self.engage_yaw = (pos.x,pos.y), pos.yaw
        self.target = list(run.points[0])
        self.cmd = [pos.x,pos.y,max(pos.z,self._floor())]
        self.heading.reset(pos.yaw)
        self.mode, self.reason, self.was_offboard, self.last_step = ENGAGED, '', False, now
        self.queued_course, self.course_radius = run, radius
        self.course_run = None
        self.launch_state, self.launch_reason = 'waiting_arm', 'Prepared: ARM starts takeoff and the '+run.source+' course'
        self.launch_armed_at = self.launch_settle_at = None
        self.log(now, self.launch_reason)
        return True, self.launch_reason

    def prepare_takeoff_course(self, now, path, radius, pos, fc, depth=None):
        """Prepare vertical takeoff only; an unchecked route is never executable.

        Ground geometry establishes the height and reference. After a settled
        hover, the node must supply a fresh checked path to start the course.
        """
        if self.p.obstacle_mode in ('shadow', 'ego'):
            return self.prepare_course(now, path, radius, pos, fc, None)
        if self.paused_course is not None or self.deferred_course is not None or self.queued_course is not None or (self.course_run and self.course_run.active):
            return False, 'course is already prepared or running'
        if self.mode not in (IDLE, ENGAGED) or self.was_offboard:
            return False, 'reset on the launch mark before preparing takeoff'
        if not self.link_alive(now):
            return False, 'dashboard heartbeat lost'
        if (pos is None or not pos.valid or not 0 <= now-pos.t <= self.p.pos_timeout_s
                or not all(math.isfinite(v) for v in (pos.x,pos.y,pos.z,pos.yaw))
                or fc is None or not fc.px4_link or not 0 <= now-fc.t <= self.p.fc_timeout_s
                or not math.isfinite(fc.origin_z)):
            return False, 'fresh position and flight status are required'
        if fc.armed or not fc.landed:
            return False, 'prepare automatic takeoff while disarmed and landed'
        if (any(v is None or not math.isfinite(v) for v in (pos.vx,pos.vy,pos.vz))
                or math.hypot(pos.vx,pos.vy) > .15 or abs(pos.vz) > .10):
            return False, 'fresh settled XYZ velocity is required before automatic takeoff'
        if abs(fc.origin_z-pos.z) > self.p.floor_tol_m:
            return False, 'flight interface and PX4 ground references disagree'
        try:
            anchor = path['anchor']
            if (self.home is None or math.dist(self.home[:3], anchor['origin_enu']) > .02
                    or abs(self.home[3]-anchor['yaw_rad']) > .001):
                raise ValueError('home and course reference disagree; reset on the launch mark')
            # Validate every height/radius and the original start, but retain
            # no unchecked follower or horizontal flight commands.
            geometry = CourseRun(path, radius, self.p.course_v_xy, self.p.course_v_z,
                                max(fc.origin_z,pos.z)+self.p.min_alt_m, pos.z+self._top_m())
            first = list(geometry.points[0])
            if (math.dist((pos.x,pos.y), first[:2]) > .05
                    or math.dist((pos.x,pos.y,pos.z), self.home[:3]) > .10):
                raise ValueError('place the drone on the course launch mark before preparing')
            target = [pos.x,pos.y,first[2]]
            guard = ObstacleGuard({'clearance_m': .30})
            try:
                observations(depth)
            except (ValueError, TypeError):
                pass  # Unknown space permits only the explicit vertical takeoff.
            else:
                guard.check(depth, (pos.x,pos.y,pos.z), (pos.vx,pos.vy),
                            ('pose', target), (pos.x,pos.y,pos.z), .1)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            return False, str(exc)
        self.origin_z, self.ground_z = fc.origin_z, pos.z
        self.engage_xy, self.engage_yaw = (pos.x,pos.y), pos.yaw
        self.target = target
        self.cmd = [pos.x,pos.y,max(pos.z,self._floor())]
        self.heading.reset(pos.yaw)
        self.mode, self.reason, self.was_offboard, self.last_step = ENGAGED, '', False, now
        self.course_run, self.course_radius = None, None
        self.deferred_course = dict(reference=path['reference'], radius=radius,
                                    takeoff_point=target, settled=False, guard=guard)
        self.launch_state = 'waiting_arm'
        self.launch_reason = 'Take off vertically, then hold until a checked course is available'
        self.launch_armed_at = self.launch_settle_at = None
        self.log(now, self.launch_reason)
        return True, self.launch_reason

    def _advance_deferred_launch(self, now, pos, fc):
        pending = self.deferred_course
        if self.mode != ENGAGED:
            self.stop_course(now, 'Course launch cancelled: '+(self.reason or self.mode))
            return
        if self.launch_armed_at is not None and not fc.armed:
            self._trip(now, 'disarmed during course launch')
            return
        if not fc.armed:
            return
        if self.launch_armed_at is None:
            self.launch_armed_at = now
        if pending['settled'] and fc.landed and not self.p.no_holds:
            self._trip(now, 'course launch interrupted by landing')
            return
        if not pending['settled'] and now-self.launch_armed_at > 30. and not self.p.no_holds:
            self._trip(now, 'course takeoff did not settle within 30 seconds')
            return
        if not fc.offboard_active:
            if not fc.landed:
                self._trip(now, 'manual takeoff before course Offboard handover')
            else:
                self.launch_state, self.launch_reason = 'waiting_offboard', 'Waiting for pilot Offboard authority'
            return
        settled = (self.was_offboard and not fc.landed and
            math.dist((pos.x,pos.y,pos.z), pending['takeoff_point']) <= .10 and
            math.dist(self.cmd,pending['takeoff_point']) <= .02 and
            all(v is not None and math.isfinite(v) for v in (pos.vx,pos.vy,pos.vz)) and
            math.hypot(pos.vx,pos.vy) <= .15 and abs(pos.vz) <= .10)
        if not settled:
            self.launch_settle_at = None
            self.launch_state = 'settling' if pending['settled'] else 'taking_off'
            self.launch_reason = 'Settling in a hover before checking the course' if pending['settled'] else 'Taking off vertically to the course height'
            return
        if self.launch_settle_at is None:
            self.launch_settle_at = now
        if now-self.launch_settle_at < .5:
            self.launch_state, self.launch_reason = 'settling', 'Settling before checking the course'
            return
        pending['settled'] = True
        if self.launch_state != 'waiting_route':
            self.launch_reason = 'Holding position; waiting for a fresh obstacle-checked route'
        self.launch_state = 'waiting_route'

    def start_deferred_course(self, now, path, pos, fc, depth, velocity_limit_mps=1.):
        """Promote a settled takeoff once, using only a current guarded route."""
        pending = self.deferred_course
        if pending is None or self.launch_state != 'waiting_route':
            return False, 'takeoff is not waiting for a route'
        try:
            if path.get('reference') != pending['reference']:
                raise ValueError('course reference changed during takeoff')
            if path.get('avoidance') is None:
                raise ValueError('automatic course requires an obstacle-checked path')
            if (pos.vz is None or not math.isfinite(pos.vz) or abs(pos.vz) > .10
                    or math.dist((pos.x,pos.y,pos.z), path['points'][0]) > .15
                    or math.dist(self.cmd,path['points'][0]) > .05):
                raise ValueError('holding at takeoff point; checked route must start here')
            guard = ObstacleGuard(path['avoidance'])
            guard.inherit(pending['guard'])
            if _live(depth) or self.p.require_checked_route:    # no depth now: the in-flight guard takes over
                guard.check(depth, (pos.x,pos.y,pos.z), (pos.vx,pos.vy),
                            ('pose',self.cmd), self.cmd, .1)
            # Preserve every occupied cell seen during this launch when the
            # checked course takes over. Disappearing observations do not clear it.
            path = dict(path, avoidance=dict(path['avoidance'],
                        walls=list(guard.cells.values()), cell_m=guard.cell_m))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            return False, str(exc)
        self.deferred_course = None
        ok, reason = self.start_course(now, path, pending['radius'], pos, fc, velocity_limit_mps)
        if not ok:
            self.deferred_course = pending
            return ok, reason
        self.launch_state, self.launch_reason = 'flying', 'Takeoff complete; following the checked course'
        return True, self.launch_reason

    def _advance_launch(self, now, pos, fc):
        if self.deferred_course is not None:
            self._advance_deferred_launch(now, pos, fc)
            return
        run = self.queued_course
        if run is None:
            return
        if self.mode != ENGAGED:
            self.stop_course(now, 'Course launch cancelled: '+(self.reason or self.mode))
            return
        if self.launch_armed_at is not None and not fc.armed:
            self._trip(now, 'disarmed during course launch')
            return
        if not fc.armed:
            return
        if self.launch_armed_at is None:
            self.launch_armed_at = now
        if now-self.launch_armed_at > 30. and not self.p.no_holds:
            self._trip(now, 'course takeoff did not settle within 30 seconds')
            return
        if not fc.offboard_active:
            if not fc.landed:
                self._trip(now, 'manual takeoff before course Offboard handover')
                return
            self.launch_state, self.launch_reason = 'waiting_offboard', 'Waiting for pilot Offboard authority'
            return
        self.launch_state, self.launch_reason = 'taking_off', 'Taking off to the first course point'
        settled = (self.was_offboard and not fc.landed and
            math.dist((pos.x,pos.y,pos.z), run.points[0]) <= .10 and
            math.dist(self.cmd,run.points[0]) <= .02 and
            all(v is not None and math.isfinite(v) for v in (pos.vx,pos.vy,pos.vz)) and
            math.hypot(pos.vx,pos.vy) <= .15 and abs(pos.vz) <= .10)
        if not settled:
            self.launch_settle_at = None
            return
        self.launch_state, self.launch_reason = 'settling', 'Settling before starting the '+run.source+' course'
        if self.launch_settle_at is None:
            self.launch_settle_at = now
        if now-self.launch_settle_at < .5:
            return
        self.queued_course, self.course_run = None, run
        self.launch_state, self.launch_reason = 'flying', 'Takeoff complete; following the '+run.source+' course'
        self.heading.hold()
        self.log(now, self.launch_reason)

    def start_course(self, now, path, radius, pos, fc, velocity_limit_mps=1.):
        if self.paused_course is not None or self.deferred_course is not None or self.queued_course is not None or (self.course_run is not None and self.course_run.active):
            return False, 'course is already running'
        if (self.mode != ENGAGED or not self.was_offboard or fc is None
                or not fc.px4_link or now-fc.t > self.p.fc_timeout_s
                or not fc.armed or not fc.offboard_active or fc.landed):
            return False, 'Start course requires a prepared, airborne Offboard hover'
        if (pos is None or not pos.valid or now-pos.t > self.p.pos_timeout_s
                or not all(math.isfinite(v) for v in (pos.x,pos.y,pos.z,pos.yaw))):
            return False, 'no fresh PX4 position'
        if not self.link_alive(now):
            return False, 'dashboard heartbeat lost'
        if (pos.vx is None or pos.vy is None or not math.isfinite(pos.vx) or not math.isfinite(pos.vy)
                or math.hypot(pos.vx,pos.vy) > .15):
            return False, 'settle in a hover before starting the course'
        anchor = path['anchor']
        origin = anchor['origin_enu']
        if (self.home is None or math.dist(self.home[:3], origin) > .02
                or abs(self.home[3]-anchor['yaw_rad']) > .001
                or abs(self.ground_z-origin[2]) > .10):
            return False, 'home and course reference disagree; reset on the launch mark'
        try:
            run = self._make_course(path, radius, velocity_limit_mps)
        except (ValueError, TypeError, OverflowError, ImportError) as exc:
            return False, str(exc)
        if (math.dist((pos.x,pos.y,pos.z),run.points[0]) > .15
                or math.dist(self.cmd,run.points[0]) > .05):
            return False, 'hover at the first course point (1 m above the launch mark)'
        self.course_run, self.course_radius = run, radius
        self.heading.hold()  # Start with one measured alignment to this route.
        self.course_yaw_rate=0.
        self.engage_xy = tuple(origin[:2])
        self.last_step = now
        self.log(now, f'Course started; radius {radius:g} m, speed limit {run.xy_speed:g} m/s')
        if run.controller == 'velocity_tracker':
            self.log(now, f'Initial course heading {math.degrees(run.desired_heading):.1f} deg; '
                     'holding position until measured yaw settles')
        return True, 'Course started; following the validated '+run.source+' route with '+self.course_controller

    # ------------------------------------------------------------------ helpers
    def log(self, now, text):
        self.events.append((round(now, 2), text))
        del self.events[:-30]

    def link_alive(self, now):
        return self.owner is not None and now - self.owner_hb <= self.p.hb_timeout_s

    def _floor(self):                   # what the interface will clamp to, in setpoint coordinates
        return max(self.origin_z, self.ground_z) + self.p.min_alt_m

    def _top_m(self):
        """Height limit above the ground: 2.0 m, or the hop ceiling while the return hop runs."""
        hopping = self.hop is not None and self.hop['state'] in ('ready', 'flying')
        return min(self.p.max_alt_m, self.p.hop_ceiling_m if hopping else self.p.course_ceiling_m)

    def _ceiling(self):
        return self.ground_z + self._top_m()

    def _fence(self, x, y, z):
        ex, ey = self.engage_xy
        dx, dy = x - ex, y - ey
        r = math.hypot(dx, dy)
        if r > self.fence_radius():
            k = self.fence_radius() / r
            x, y = ex + dx * k, ey + dy * k
        return [x, y, min(max(z, self._floor()), self._ceiling())]

    def _trip(self, now, why):
        if self.mode in (ENGAGED, LANDING):
            self.stop_course(now, 'Course cancelled: ' + why)
            self.mode, self.reason = FAILSAFE, why
            self.log(now, f"FAILSAFE: {why} - dashboard stopped; flight interface holds, no mode change")

    # ------------------------------------------------------------------ inputs
    def heartbeat(self, now, client):
        """Every dashboard poll is a heartbeat; only the owner's keeps control alive."""
        if client == self.owner:
            self.owner_hb = now
            return True
        return False

    def command(self, now, client, op, pos: Pos | None = None, fc: Fc | None = None, **kw):
        previous_owner = self.owner
        result = self._command(now, client, op, pos, fc, **kw)
        if result[0] and (op in ('hold','land','nudge','goto','alt','release','reset','disarm','engage')
                          or (op == 'arm' and (self.deferred_course is not None or self.paused_course is not None))
                          or (op == 'claim' and previous_owner != client)):
            self.stop_course(now, 'Course cancelled by ' + op, hold=(op == 'claim'))
        return result

    def _command(self, now, client, op, pos: Pos | None = None, fc: Fc | None = None, **kw):
        """Returns (ok, message). Only the owner may command (claim is how you become owner)."""
        if not isinstance(client, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", client):
            return False, "invalid client ID"
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in kw.values()):
            return False, "commands require finite numbers"
        if op == "alt" and "alt" not in kw:
            return False, "missing altitude"
        if op in ("engage", "alt"):
            alt = float(kw.get("alt", self.p.default_alt_m))
            if not self.p.min_alt_m <= alt <= self._top_m():
                return False, (f"height must be {self.p.min_alt_m:g}–{self._top_m():g} m; "
                               f"requested {alt:g} m")
        if op == "goto" and not {"forward", "left"} <= kw.keys():
            return False, "enter forward and left coordinates in metres"
        if op == "claim":
            if self.owner in (None, client) or now - self.owner_hb > self.p.owner_stale_s:
                if self.owner not in (None, client):
                    self.log(now, f"control taken over from a silent dashboard by {client[:6]}")
                self.owner, self.owner_hb = client, now
                return True, "you have control"
            return False, "another dashboard has control"
        if client != self.owner:
            return False, "not in control (press Take control)"
        self.owner_hb = now
        if op == "release":
            if self.mode in (ENGAGED, LANDING):
                self.log(now, "dashboard stopped; flight interface holds, no mode change")
            self.mode, self.reason = IDLE, "released"
            return True, "released"
        if op == "reset":
            if pos is None or not pos.valid or now - pos.t > self.p.pos_timeout_s:
                return False, "no fresh PX4 position for home"
            if fc is None or not fc.px4_link or now - fc.t > self.p.fc_timeout_s:
                return False, "no fresh PX4 status for home"
            if fc.armed or not fc.landed:
                return False, "reset home only while disarmed and landed"
            self.home = (pos.x, pos.y, pos.z, pos.yaw)
            self.engage_xy, self.engage_yaw = (pos.x, pos.y), pos.yaw
            self.ground_z, self.origin_z = pos.z, fc.origin_z
            self.target = [pos.x, pos.y, pos.z]
            self.cmd = list(self.target)
            self.heading.reset(pos.yaw)
            self.course_radius = None
            self.course_run = None
            self.last_out, self.was_offboard = None, False
            self.mode, self.reason = IDLE, "home reset"
            self.log(now, "home reset: current position and heading are zero on the dashboard")
            return True, "home reset"
        if op in ("arm", "disarm"):
            return True, f"{op} validated"
        if op == "engage":
            if self.mode != IDLE:
                return False, f"engage only from idle (now {self.mode}; reset first)"
            if pos is None or not pos.valid or now - pos.t > self.p.pos_timeout_s:
                return False, "no valid PX4 local position"
            if fc is None or now - fc.t > self.p.fc_timeout_s or not fc.px4_link:
                return False, "no PX4 link"
            if fc.armed:
                return False, "engage while disarmed"
            if not fc.landed:
                return False, "engage on the ground (PX4 must report landed)"
            iface_floor = fc.origin_z + self.p.min_alt_m
            if abs(iface_floor - (pos.z + self.p.min_alt_m)) > self.p.floor_tol_m:
                return False, (f"flight_interface floor is {iface_floor - pos.z:+.2f} m from the ground, not "
                               f"+{self.p.min_alt_m:.2f}: flight interface and PX4 ground references disagree")
            self.origin_z, self.ground_z = fc.origin_z, pos.z
            self.engage_xy, self.engage_yaw = (pos.x, pos.y), pos.yaw
            if self.home is None:
                self.home = (pos.x, pos.y, pos.z, pos.yaw)
            alt = float(kw.get("alt", self.p.default_alt_m))
            self.target = self._fence(pos.x, pos.y, self.ground_z + alt)
            self.cmd = [pos.x, pos.y, max(pos.z, self._floor())]
            self.heading.reset(pos.yaw)
            self.mode, self.reason, self.was_offboard, self.last_step = ENGAGED, "", False, now
            self.log(now, f"engaged: hover {self.target[2] - self.ground_z:.2f} m above the ground; "
                          "takeoff starts when armed in Offboard")
            return True, "engaged"
        if self.mode not in (ENGAGED, LANDING):
            return False, f"not engaged ({self.mode})"
        if op == "goto":
            if self.mode != ENGAGED:
                return False, "go to point is unavailable while landing"
            if pos is None or not pos.valid or now - pos.t > self.p.pos_timeout_s:
                return False, "no fresh PX4 position"
            if fc is None or not fc.px4_link or now - fc.t > self.p.fc_timeout_s:
                return False, "no fresh PX4 status"
            if not fc.armed or not fc.offboard_active or fc.landed:
                return False, "go to point requires an airborne Offboard hover"
            if self.home is None:
                return False, "home is not set"
            forward, left = float(kw['forward']), float(kw['left'])
            hx, hy, _, yaw = self.home
            c, s = math.cos(yaw), math.sin(yaw)
            x, y = hx + forward*c - left*s, hy + forward*s + left*c
            if math.hypot(x-self.engage_xy[0], y-self.engage_xy[1]) > self.fence_radius() + 1e-9:
                return False, f"point is outside the {self.fence_radius():g} m movement radius"
            # Preserve height and the current ramp position. step() advances
            # toward this absolute home-relative target at the existing speed.
            self.target = [x, y, self.target[2]]
            message = f"going to {forward:.2f} m forward, {left:.2f} m left; height unchanged"
            self.log(now, message)
            return True, message
        if op == "nudge":
            if self.mode != ENGAGED:
                return False, "landing: release or let it finish"
            fwd = max(-self.p.nudge_xy_max_m, min(self.p.nudge_xy_max_m, float(kw.get("fwd", 0.0))))
            right = max(-self.p.nudge_xy_max_m, min(self.p.nudge_xy_max_m, float(kw.get("right", 0.0))))
            up = max(-self.p.nudge_z_max_m, min(self.p.nudge_z_max_m, float(kw.get("up", 0.0))))
            c, s = math.cos(self.engage_yaw), math.sin(self.engage_yaw)
            x = self.target[0] + fwd * c + right * s
            y = self.target[1] + fwd * s - right * c
            self.target = self._fence(x, y, self.target[2] + up)
            return True, "target moved"
        if op == "alt":
            if self.mode != ENGAGED:
                return False, "landing"
            self.target = self._fence(self.target[0], self.target[1], self.ground_z + float(kw.get("alt")))
            return True, "altitude set"
        if op == "hold":
            if self.mode != ENGAGED:
                return False, "landing"
            self.target = list(self.cmd)
            self.heading.hold()
            return True, "holding"
        if op == "land":
            self._begin_landing(now, "landing: down to the floor, then a slow twist to touchdown")
            return True, "landing"
        return False, f"unknown op {op}"

    def _begin_landing(self, now, message):
        self.heading.hold()
        self.mode, self.land_phase = LANDING, "down"
        self.target = [self.cmd[0], self.cmd[1], self._floor()]
        self.log(now, message)

    def _land_if_complete(self, now, run):
        # Only a finished course lands; stopped/blocked/cancelled runs keep holding.
        if self.hop is not None and self.mode == ENGAGED:
            if run.state == 'complete' and self.hop['state'] == 'armed':
                self.hop.update(state='land', t0=now)
                self.px4_land_request = now        # node: PX4 Land mode (hold here until PX4 takes over)
                self.heading.hold()
                self.log(now, 'Course complete; PX4 Land, then Take off · return hop')
                return
            if self.hop['state'] == 'flying' and run.state == 'complete':
                self.hop.update(state='land_final', t0=now)   # hop done: PX4 lands it too
                self.px4_land_request = now
                self.heading.hold()
                self.log(now, 'Return hop complete; PX4 Land')
                return
            if self.hop['state'] == 'flying':
                self.hop = None              # hop stopped: hold as for any course
        if self.p.land_after_course and run.state == 'complete' and self.mode == ENGAGED:
            self._begin_landing(now, "Course complete; landing: down to the floor, then touchdown")

    # ------------------------------------------------------------------ the 10 Hz step
    def step(self, now, pos: Pos | None, fc: Fc | None, depth=None):
        self._last_pos = pos            # rejoin: where the camera is looking
        """Returns ('pose', (x,y,z)), ('twist', (vx,vy,vz)) or None (publish nothing)."""
        dt = 0.0 if self.last_step is None else min(max(now - self.last_step, 0.0), 0.2)
        previous_command = list(self.cmd)
        self.last_step = now
        out = None
        self.course_yaw_rate=0.
        if self.mode in (ENGAGED, LANDING):
            if not self.link_alive(now) and not self.p.no_holds:
                self._trip(now, "dashboard heartbeat lost")
            elif pos is None or not pos.valid or now - pos.t > self.p.pos_timeout_s:
                self._trip(now, "PX4 local position lost")
            elif fc is None or now - fc.t > self.p.fc_timeout_s or not fc.px4_link:
                self._trip(now, "PX4 link lost")
            elif self.was_offboard and not fc.offboard_active:
                self.mode, self.reason = PILOT, "pilot took over"
                self.log(now, "Offboard ended (pilot took over) - dashboard stopped")
            elif self.was_offboard and not fc.armed:
                self.mode, self.reason = IDLE, "disarmed"
                self.log(now, "disarmed - dashboard idle")
            elif not self.p.no_holds and (math.hypot(pos.x - self.engage_xy[0], pos.y - self.engage_xy[1])
                  > self.fence_radius() + self.p.fence_margin_m or pos.z > self._ceiling() + 0.3):
                self._trip(now, "vehicle left the fence")
        self._advance_launch(now, pos, fc)
        if self.hop is not None and self.hop['state'] in ('land', 'land_final'):
            if fc is not None and fc.landed and (not fc.armed or self.hop['state'] == 'land'):
                final = self.hop['state'] == 'land_final'
                self.hop = None if final else dict(self.hop, state='ready')
                self.mode, self.reason, self.was_offboard = IDLE, ('landed' if final else 'landed for the return hop'), False
                self.log(now, 'Landed; course + return hop done' if final else 'Landed; press Take off · return hop')
            elif (self.mode == ENGAGED and fc is not None and fc.offboard_active
                  and now - self.hop['t0'] > self.p.hop_px4_land_wait_s):
                self.hop['t0'] = float('inf')      # once: PX4 did not take the Land request
                self._begin_landing(now, 'PX4 did not switch to Land; dashboard landing instead')
        paused = self.paused_course
        if paused is not None:
            if self.mode != ENGAGED:
                self.stop_course(now,'Replan cancelled: '+(self.reason or self.mode))
            elif fc.landed and not self.p.no_holds:
                self._trip(now,'Replan cancelled after landing')
            else:
                if (any(v is None or not math.isfinite(v) for v in (pos.vx,pos.vy,pos.vz))
                        or math.hypot(pos.vx,pos.vy) > .15 or abs(pos.vz) > .10
                        or math.dist(self.cmd,(pos.x,pos.y,pos.z)) > .10):
                    paused['settled_at'] = None
                try:
                    walls,cell,_ = observations(depth)
                except (ValueError,TypeError):
                    pass
                else:
                    try:
                        paused['guard']._remember(walls,cell)
                    except ValueError as exc:
                        self._trip(now,str(exc))
        pending = self.deferred_course
        run = self.course_run
        previous_path_time = None if run is None else run.elapsed
        guarded = self.queued_course or (run if run is not None and run.active else None)
        if run is not None and run.active:
            if self.mode != ENGAGED:
                self.stop_course(now, 'Course cancelled: ' + (self.reason or self.mode))
            elif fc.landed and not self.p.no_holds:
                self._trip(now, 'course interrupted by landing')
            elif self.p.obstacle_mode != 'ego' and not self.p.no_holds and self._tracking_error(run, pos) > .5:
                self._trip(now, 'course tracking error exceeds 0.5 m')
            elif self.p.obstacle_mode == 'ego':
                self._ego_target(now, run, pos)
            else:
                self._pass_through(now, run, pos, depth, dt)
                if run.controller!='velocity_tracker':
                    self.target = run.target(dt, self.cmd, (pos.x,pos.y,pos.z),pace=self.heading.pace)
        if self.mode in (ENGAGED, LANDING):
            if fc.offboard_active and fc.armed and not self.was_offboard:
                self.was_offboard = True
                self.log(now, "Offboard active - dashboard is flying")
            if not self.was_offboard:
                # waiting for the pilot: keep the stream alive at the current spot (floored), no ramp,
                # so Offboard starts with only the interface's 0.3 m lift
                self.cmd = [self.engage_xy[0], self.engage_xy[1], max(pos.z, self._floor())]
                self.heading.reset(pos.yaw)
                out = ("pose", tuple(self.cmd))
            elif self.mode == LANDING and self.land_phase == "touch":
                if fc.landed or now - self.land_t0 > self.p.land_timeout_s:
                    why = "touchdown" if fc.landed else "touchdown timeout"
                    self.mode, self.reason = IDLE, why
                    self.log(now, f"landing done ({why}) - setpoints stopped; pilot disarms")
                else:
                    out = ("twist", (0.0, 0.0, -self.p.land_v))
            elif run is not None and run.active and run.controller=='velocity_tracker':
                speed=(math.hypot(pos.vx,pos.vy) if pos.vx is not None and pos.vy is not None
                       and math.isfinite(pos.vx) and math.isfinite(pos.vy) else None)
                self.cmd=[pos.x,pos.y,pos.z]
                self.heading.reset(pos.yaw)
                if speed is None:
                    self._trip(now,'horizontal velocity missing during velocity tracking')
                else:
                    try:
                        velocity,yaw_rate=run.step(dt,self.cmd,pos.yaw,speed)
                        if (len(velocity)!=3 or not all(math.isfinite(v) for v in (*velocity,yaw_rate))
                                or math.sqrt(sum(v*v for v in velocity))>run.xy_speed+1e-6
                                or abs(velocity[2])>self.p.v_z+1e-6
                                or abs(yaw_rate)>math.radians(90)+1e-6):
                            raise ValueError('velocity tracker exceeded its command bounds')
                        self.target=list(run.reference_position)
                        self.course_yaw_rate=float(yaw_rate)
                        self.heading.phase=run.phase
                        self.heading.reason=run.reason
                        self.heading.goal=run.desired_heading
                        self.heading.error=math.atan2(math.sin(run.desired_heading-pos.yaw),
                                                     math.cos(run.desired_heading-pos.yaw))
                        out=('twist',tuple(float(v) for v in velocity))
                        if getattr(run, 'position_hold', None) is not None:
                            self.cmd=list(run.position_hold)
                            self.target=list(self.cmd)
                            self.heading.yaw=run.commanded_heading
                            self.course_yaw_rate=0.
                            out=('pose',tuple(self.cmd))
                        if not run.active:
                            self.cmd=[pos.x,pos.y,pos.z]
                            self.target=list(self.cmd)
                            self.course_yaw_rate=0.
                            self.heading.reset(pos.yaw)
                            out=('pose',tuple(self.cmd))
                            self.log(now,run.reason)
                            self._land_if_complete(now, run)
                    except Exception as exc:
                        self._trip(now,'velocity tracker stopped: '+str(exc))
                        out=None
            else:
                d = [t - c for t, c in zip(self.target, self.cmd)]
                speed = (math.hypot(pos.vx,pos.vy) if pos.vx is not None and pos.vy is not None
                         and math.isfinite(pos.vx) and math.isfinite(pos.vy) else None)
                error = math.hypot(self.cmd[0]-pos.x,self.cmd[1]-pos.y)
                if run is not None and run.active and run.pending is None:
                    # The course clock is waiting for tracking to catch up.
                    # This is not an explicit Hold or a stationary route segment:
                    # preserve alignment, and finish only the previous yaw turn.
                    self.heading.pause_advance(dt)
                    advance=False
                elif fc.landed and math.hypot(d[0],d[1])>1e-6:
                    self.heading.phase='waiting'
                    self.heading.reason='Waiting for takeoff before turning'
                    advance=False
                else:
                    self.heading.fixed = self.p.heading_mode == 'hold_start'
                    advance=self.heading.advance_allowed(d[0],d[1],pos.yaw,speed,error,dt,
                        continuous=bool(run is not None and run.active))
                if advance:
                    xy_limit=run.xy_speed if run is not None and run.active else self.p.v_xy
                    sx,sy=limited_horizontal_step(d[0],d[1],self.cmd,(pos.x,pos.y),dt,xy_limit)
                    self.cmd[0] += sx
                    self.cmd[1] += sy
                vz = (self.p.course_v_z if (run is not None and run.active) or self.queued_course is not None
                      or self.deferred_course is not None else self.p.v_z)
                self.cmd[2] += max(-vz * dt, min(vz * dt, d[2]))
                out = ("pose", tuple(self.cmd))
                if run is not None and run.active:
                    run.observe(self.cmd, (pos.x,pos.y,pos.z), speed,
                                heading_settled=self.heading.course_heading_settled,
                                advance_allowed=advance)
                    if not run.active:
                        self.target = list(self.cmd)
                        self.heading.hold()
                        self.log(now, run.reason)
                        self._land_if_complete(now, run)
                if (self.mode == LANDING and self.land_phase == "down"
                        and abs(self.cmd[2] - self._floor()) < 0.02 and pos.z < self._floor() + 0.15):
                    self.land_phase, self.land_t0 = "touch", now
                    self.log(now, "at the floor - final descent")
        if out is not None and pending is not None:
            try:
                observations(depth)
            except (ValueError, TypeError):
                pass  # No horizontal course may start without observations.
            else:
                try:
                    pending['guard'].check(depth, (pos.x,pos.y,pos.z), (pos.vx,pos.vy), out,
                                           previous_command, dt)
                except (ValueError, TypeError, KeyError, OverflowError) as exc:
                    self._trip(now, 'Takeoff obstacle guard: '+str(exc))
                    self.cmd = self.target = [pos.x,pos.y,pos.z]
                    self.heading.reset(pos.yaw)
                    out = (('pose',tuple(self.cmd)) if fc.armed and fc.offboard_active else None)
        if out is not None and guarded is not None and guarded.avoidance is not None:
            try:
                guarded.obstacle_guard.check(depth, (pos.x,pos.y,pos.z), (pos.vx,pos.vy), out,
                                             previous_command, dt, now=now,
                                             route=route_ahead(guarded, (pos.x,pos.y), guarded.xy_speed),
                                             height_route=guarded.points, camera=self._camera_pose(),
                                             here=(pos.x, pos.y, pos.z))
                passing = getattr(getattr(guarded, 'pass_through', None), 'state', None) in ('hover', 'commit')
                if (guarded is run and self.mode == ENGAGED and self.nudge_request is None and not passing
                        and now >= self.nudge_retry_at and fc.armed and not fc.landed):
                    ahead = route_ahead(guarded, (pos.x,pos.y), guarded.xy_speed, margin_m=NUDGE_LOOKAHEAD_M)
                    if guarded.obstacle_guard.route_conflict(ahead, NUDGE_TRIGGER_M, height_route=guarded.points):
                        self.nudge_request = dict(run=guarded, at=now, path=None,
                            trigger=guarded.obstacle_guard.conflict_cells(ahead, NUDGE_TRIGGER_M,
                                                                          height_route=guarded.points),
                            trigger_version=guarded.obstacle_guard._last_version)
                        self.replan_reason = 'Obstacle ahead on the route; nudging without stopping'
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                reason = 'Obstacle guard: '+str(exc)
                can_replan = (guarded is run and self.mode == ENGAGED
                    and fc.armed and fc.offboard_active and not fc.landed
                    and str(exc) in ('commanded position crosses obstacle clearance',
                                    'obstacle inside the commanded or measured stopping path'))
                if can_replan:
                    # Remember what stopped us even if seen in one frame only, so a
                    # replan from memory (during depth dropouts) still avoids it.
                    guarded.obstacle_guard.remember_last(near=(pos.x, pos.y))
                    guarded.stop('paused',reason)
                    self.paused_course = dict(run=guarded,guard=guarded.obstacle_guard,
                        reference=guarded.reference,controller=guarded.controller,
                        elapsed_s=previous_path_time,settled_at=None)
                    self.replan_reason = 'Holding; checking a route around the obstacle'
                    self.reason = ''
                    self.log(now,reason+'; paused for replanning')
                else:
                    self._trip(now,reason)
                    guarded.stop('blocked',self.reason)
                self.cmd = self.target = [pos.x,pos.y,pos.z]
                self.heading.reset(pos.yaw)
                self.course_yaw_rate = 0.
                # Send one immediate current-position brake while authority
                # and pose are fresh. Waiting for a velocity timeout could
                # otherwise keep the previous moving command alive for 1 s.
                # Subsequent FAILSAFE ticks publish nothing; no auto-resume.
                out = (('pose', tuple(self.cmd)) if fc.armed and fc.offboard_active
                       and all(math.isfinite(v) for v in (*self.cmd, pos.yaw)) else None)
        self.last_out = out
        return out

    def status(self, now):
        return {
            "mode": self.mode, "reason": self.reason, "owner": self.owner,
            "obstacle_mode": self.p.obstacle_mode,
            "course_replan": dict(active=self.paused_course is not None,
                                  state='waiting_route' if self.paused_course is not None else 'idle',
                                  reason=self.replan_reason),
            "course_run": None if self.course_run is None else self.course_run.status(),
            "course_launch": dict(active=self.queued_course is not None or self.deferred_course is not None or self.paused_course is not None, state=self.launch_state,
                                  reason=self.launch_reason),
            "course_controller": {"selected": self.course_controller},
            "hop": None if self.hop is None else dict(state=self.hop['state']),
            "heading": dict(self.heading.status(), policy=self.p.heading_mode),
            "link_age_s": None if self.owner is None else round(now - self.owner_hb, 2),
            "link_alive": self.link_alive(now), "was_offboard": self.was_offboard,
            "engage_xy": [round(v, 3) for v in self.engage_xy],
            "target": [round(v, 3) for v in self.target], "cmd": [round(v, 3) for v in self.cmd],
            "target_alt": round(self.target[2] - self.ground_z, 2), "origin_z": round(self.origin_z, 3),
            "ground_z": round(self.ground_z, 3),
            "home": None if self.home is None else {"x": round(self.home[0], 3), "y": round(self.home[1], 3),
                "z": round(self.home[2], 3), "yaw_deg": round(math.degrees(self.home[3]), 1)},
            "out": None if self.last_out is None else [self.last_out[0], [round(v, 3) for v in self.last_out[1]]],
            "land_phase": self.land_phase if self.mode == LANDING else "",
            "events": self.events[-12:], "params": {**{k: getattr(self.p, k) for k in
                                                    ("hb_timeout_s", "min_alt_m", "max_alt_m", "v_xy", "course_v_xy", "v_z",
                                                     "course_ceiling_m", "hop_ceiling_m")},
                                                    "fence_r_m": self.fence_radius()},
        }

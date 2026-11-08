"""Carry-it dry run of a course. No ROS, no publishers.

The full autopilot logic runs on a private copy of the flight core: real pose and real depth
go in; preview path, pass-through centring, nudges, guard stops and replans all run exactly
as in flight. Nothing it computes is ever sent to PX4. The course clock waits for the
aircraft (the follower only advances while the carrier is within ~0.2 m of the command),
so you walk the drone along and the dashboard shows where the autopilot wants it next.

The dashboard creates one only while disarmed and stops it the moment PX4 reports armed.
"""
import copy
import math

from course_replan import plan_remaining
from dash_core import DashCore, Fc, Pos

START_TOLERANCE_M = .40     # hold the drone this close to the course start to begin
STRAY_M = .45               # carrier farther than this from the dot: freeze the course, ask to come back
OWNER = 'dry-run'


class DryRun:
    def __init__(self, now, core, path, radius, pos):
        if core.home is None:
            raise ValueError('Set home / course start first')
        if pos is None or not pos.valid:
            raise ValueError('no fresh position')
        first = [float(v) for v in path['points'][0]]
        away = math.dist((pos.x, pos.y, pos.z), first)
        if away > START_TOLERANCE_M:
            raise ValueError(f'Hold the drone at the course start ({away:.2f} m away; '
                             f'the start is {first[2]-core.home[2]:.2f} m above home)')
        params = copy.deepcopy(core.p)
        params.land_after_course = False
        d = DashCore(params)
        d.owner, d.owner_hb = OWNER, now
        d.home = tuple(core.home)
        d.ground_z = d.origin_z = float(path['anchor']['origin_enu'][2])
        d.mode, d.was_offboard = 'engaged', True
        d.course_controller = core.course_controller
        d.cmd, d.target = list(first), list(first)
        d.heading.reset(pos.yaw)
        d.last_step = now
        # Start exactly as an airborne hover at the first point would (only the start check
        # uses this pose; every tick after uses the real one).
        ok, why = d.start_course(now, path, radius, Pos(now, *first, pos.yaw, True, 0., 0., 0.),
                                 self._fc(now, d), velocity_limit_mps=1.)
        if not ok:
            raise ValueError(why)
        self.core, self.state, self.reason = d, 'running', 'Carry the drone along the green route'
        self.started, self.nudges, self.pauses = now, 0, 0
        self.replan, self._paused = None, False
        self._t, self._yaw = None, None

    @staticmethod
    def _fc(now, core):
        return Fc(now, True, True, True, False, origin_z=core.origin_z)

    def stop(self, reason):
        if self.state == 'running':
            self.state, self.reason = 'stopped', reason

    def tick(self, now, pos, depth):
        """Flight-loop tick (under the node lock): step the private core with the real pose."""
        if self.state != 'running' or pos is None or not pos.valid:
            return
        d = self.core
        d.heartbeat(now, OWNER)
        run = d.course_run
        ref = getattr(run, 'reference_position', None) if run is not None else None
        point = ref if ref is not None else d.cmd
        # EGO mode: the dot is EGO's own time-based trajectory, which runs up to the carrot (2 m) ahead
        # and waits there; freezing at 0.45 m would never let the carrier catch it.
        stray = STRAY_M if d.p.obstacle_mode != 'ego' else 2.5
        if math.dist((pos.x, pos.y, pos.z), point) > stray:
            d.last_step = now                # frozen: no clock, no tracking-error trip
            self.reason = f'Come back to the green dot ({math.dist((pos.x, pos.y, pos.z), point):.2f} m away); course paused'
            return
        self.reason = 'Carry the drone along the green route'
        # Carrying yaw is irrelevant: tell the private core the nose points where it commands.
        # The nose the private core believes in: the position follower's commanded heading; for the
        # velocity follower its absolute alignment heading, then its commanded yaw RATE integrated
        # (what a real drone would do), since its heading is reset to the measured yaw every tick.
        dt = 0. if self._t is None else max(0., min(now-self._t, .2))
        self._t = now
        if run is not None and run.controller == 'velocity_tracker':
            if getattr(run, 'phase', '') == 'initial_align' and getattr(run, 'commanded_heading', None) is not None:
                self._yaw = run.commanded_heading
            else:
                self._yaw = (pos.yaw if self._yaw is None else self._yaw)+getattr(d, 'course_yaw_rate', 0.)*dt
            yaw = self._yaw
        else:
            yaw = d.heading.yaw if d.heading.yaw is not None else pos.yaw
        d._camera_pos = pos                  # the real camera pose, for the rejoin view check
        pos = Pos(pos.t, pos.x, pos.y, pos.z, yaw, pos.valid, pos.vx, pos.vy, pos.vz)
        d.step(now, pos, self._fc(now, d), depth)
        n = d.nudge_request
        if n is not None and n.get('path') is not None and d.apply_nudge(now, n['path'], 1.)[0]:
            self.nudges += 1
        if d.paused_course is not None:
            if not self._paused:
                self.pauses += 1
            self._paused = True
            if self.replan is not None:
                d.resume_course(now, self.replan, pos, self._fc(now, d), depth, 1.)
                self.replan = None
        else:
            self._paused = False
        run = d.course_run
        if d.mode == 'failsafe':
            self.state, self.reason = 'stopped', 'Autopilot would have stopped: ' + str(d.reason)
        elif run is not None and not run.active and d.paused_course is None:
            self.state = 'finished' if run.state == 'complete' else 'stopped'
            self.reason = run.reason

    def plan_job(self, now, pos):
        """Inputs for the slow planning step (copied under the lock)."""
        d, run = self.core, self.core.course_run
        if self.state != 'running' or pos is None:
            return None
        if d.paused_course is not None:
            g, r = d.paused_course['guard'], d.paused_course['run']
            return dict(kind='replan', run=copy.deepcopy(r), elapsed=d.paused_course['elapsed_s'],
                        start=(pos.x, pos.y, pos.z), walls=g.known_walls(r.points), cell=g.cell_m,
                        clearance=g.clearance, floor=d._floor(), ceiling=d._ceiling())
        rejoin = d.rejoin_job(now)
        if rejoin is not None:
            return dict(rejoin, kind='rejoin')
        n = d.nudge_request
        if n is not None and n.get('path') is None and run is not None and run.active:
            g = run.obstacle_guard
            return dict(kind='nudge', request=n, run=copy.deepcopy(run), elapsed=run.elapsed,
                        start=tuple(d.cmd), walls=g.known_walls(run.points), cell=g.cell_m,
                        clearance=g.clearance, floor=d._floor(), ceiling=d._ceiling())
        return None

    @staticmethod
    def plan(job, depth):
        """Slow part, outside the lock. Returns (path or None, why)."""
        r = job['run']
        points, times = (job['points'], job['times']) if job['kind'] == 'rejoin' else (list(r.points), list(r.times))
        try:
            path = plan_remaining(points, times, job['elapsed'], job['start'], r.anchor,
                                  r.reference, r.radius, depth, job['walls'], known_cell_m=job['cell'],
                                  clearance_m=job['clearance'], floor=job['floor'], ceiling=job['ceiling'])
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            return None, str(exc)
        return path, None

    def commit(self, now, job, path, why):
        d = self.core
        if job['kind'] == 'rejoin':
            d.commit_rejoin(now, job, path)
        elif job['kind'] == 'replan':
            self.replan = path
        elif d.nudge_request is job['request']:
            if path is not None and path.get('source') == 'adjusted':
                job['request'].update(path=path, planned_at=now)
            else:
                d.nudge_request, d.nudge_retry_at = None, now + 1.

    def status(self, pos):
        d, run = self.core, self.core.course_run
        # Where the autopilot wants the drone now: the position follower's command, or the
        # velocity follower's reference point on its trajectory.
        ref = getattr(run, 'reference_position', None) if run is not None else None
        point = [float(v) for v in (ref if ref is not None else d.cmd)]
        out = dict(state=self.state, reason=self.reason, nudges=self.nudges, pauses=self.pauses,
                   events=[e[1] for e in d.events[-4:]], cmd=[round(v, 3) for v in point])
        if run is not None:
            st = run.status()
            out['progress'] = st.get('progress')
            pts = run.points
            step = max(1, len(pts)//200)
            out['route'] = [[round(p[0], 3), round(p[1], 3)] for p in pts[::step]]
            pt = getattr(run, 'pass_through', None)
            out['pass_through'] = None if pt is None else pt.status()
        if pos is not None and pos.valid:
            dx, dy, dz = point[0]-pos.x, point[1]-pos.y, point[2]-pos.z
            c, s = math.cos(pos.yaw), math.sin(pos.yaw)
            out['carry'] = dict(forward_m=round(c*dx+s*dy, 2), left_m=round(-s*dx+c*dy, 2), up_m=round(dz, 2))
        return out

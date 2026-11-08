"""Paced playback of a validated anchored route; no publishers or arming."""
from bisect import bisect_right
import copy
import math
from course_safety import MAX_COURSE_SPEED_MPS, ObstacleGuard


WAYPOINT_ACCEPTANCE_M = .05



def end_reached(run, position):
    """Within 15 cm of the last point; a run with end_dz_m set (course + return hop) checks 15 cm
    horizontally and only end_dz_m in height, so a biased altitude estimate cannot stall the finish."""
    end = run.points[-1]
    dz = getattr(run, 'end_dz_m', None)
    if dz is None:
        return math.dist(position, end) <= .15
    return math.dist(position[:2], end[:2]) <= .15 and abs(position[2]-end[2]) <= dz

class CourseRun:
    controller = 'smooth_position'

    def __init__(self, path, radius, xy_speed, z_speed, floor, ceiling):
        self.avoidance = copy.deepcopy(path.get('avoidance'))
        self.display = copy.deepcopy(path.get('display'))
        self.source = path.get('source', 'original')
        self.obstacle_guard = None if self.avoidance is None else ObstacleGuard(self.avoidance)
        xy_speed = min(xy_speed, MAX_COURSE_SPEED_MPS)
        if not all(math.isfinite(v) and v > 0 for v in (xy_speed, z_speed)):
            raise ValueError('course speed limits must be finite and positive')
        if type(radius) not in (int, float) or not math.isfinite(radius) or not .5 <= radius <= 20:
            raise ValueError('course radius must be 0.5–20 m')
        self.points = [tuple(p) for p in path['points']]
        if len(self.points) < 2 or any(len(p) != 3 or not all(math.isfinite(v) for v in p)
                                      for p in self.points):
            raise ValueError('course needs finite XYZ waypoints')
        self.anchor = path['anchor']
        self.reference = path['reference']
        ox, oy, _ = self.anchor['origin_enu']
        if any(math.hypot(p[0]-ox, p[1]-oy) > radius for p in self.points):
            raise ValueError('course extends beyond the selected course radius')
        if any(not floor <= p[2] <= ceiling for p in self.points):
            raise ValueError('course is outside the configured height limits')
        times = path.get('times_s')
        if times is None:
            times = [0.]
            for a, b in zip(self.points, self.points[1:]):
                times.append(times[-1] + max(.01, math.dist(a[:2], b[:2])/xy_speed,
                                            abs(a[2]-b[2])/z_speed))
        if (len(times) != len(self.points) or not all(math.isfinite(t) for t in times)
                or any(b <= a for a, b in zip(times, times[1:]))):
            raise ValueError('course times must increase and match the waypoints')
        scale = 1.
        for a, b, ta, tb in zip(self.points, self.points[1:], times, times[1:]):
            scale = max(scale, math.dist(a[:2], b[:2]) / ((tb-ta)*xy_speed),
                        abs(b[2]-a[2]) / ((tb-ta)*z_speed))
        self.times = [(t-times[0])*scale for t in times]
        if (not all(math.isfinite(t) for t in self.times)
                or any(b <= a for a, b in zip(self.times,self.times[1:]))):
            raise ValueError('course timing cannot be represented at the configured speed')
        self.radius, self.elapsed, self.pending = radius, 0., None
        self.xy_speed = xy_speed
        self.floor,self.ceiling,self.z_speed = floor,ceiling,z_speed
        self.state, self.reason = 'running', 'Following the validated '+self.source+' route'
        self.active = True

    def sample(self, t):
        i = min(bisect_right(self.times, t)-1, len(self.times)-2)
        i = max(0, i)
        u = max(0., min(1., (t-self.times[i])/(self.times[i+1]-self.times[i])))
        return [a+(b-a)*u for a, b in zip(self.points[i], self.points[i+1])]

    def target(self, dt, cmd, position, pace=1.):
        if getattr(self, 'hold_clock', False):        # hole control: hovering to measure a hole
            self.pending = None
            return self.sample(self.elapsed)
        pace = pace*getattr(self, 'pace_scale', 1.)
        # A stopped clock never skips ahead after a turn, lag, or delayed tick.
        if self.pending is None and math.dist(cmd, position) <= .20:
            # Keep a tiny pending step even at zero pace so its tangent remains
            # available to the heading tracker. The measured-heading gate must
            # allow the move before observe can commit this path time.
            increment=min(max(dt,0.),.2)*max(.05,min(float(pace),1.))
            self.pending = min(self.times[-1], self.elapsed + increment)
        return self.sample(self.elapsed if self.pending is None else self.pending)

    def observe(self, cmd, position, speed, heading_settled=True, advance_allowed=True):
        if self.pending is not None:
            remaining = math.dist(cmd, self.sample(self.pending))
            # Pass intermediate targets within 5 cm instead of waiting for an
            # exact setpoint match. Heading/lead holds must not advance the
            # clock, and the final target must still be commanded in full.
            pass_through = (advance_allowed and self.pending < self.times[-1]
                            and remaining <= WAYPOINT_ACCEPTANCE_M)
            if remaining < 1e-7 or pass_through:
                self.elapsed, self.pending = self.pending, None
        if (self.elapsed >= self.times[-1] and end_reached(self, position)
                and speed is not None and speed <= .15 and heading_settled):
            self.stop('complete', 'Course complete; holding the final position')

    def stop(self, state='stopped', reason='Course stopped; holding'):
        self.active, self.state, self.reason = False, state, reason

    def status(self):
        return dict(active=self.active, state=self.state, reason=self.reason,controller=self.controller,
                    progress=round(self.elapsed/self.times[-1], 4),
                    path_time_s=round(self.elapsed, 2), duration_s=round(self.times[-1], 2),
                    radius_m=self.radius,xy_speed_mps=self.xy_speed,
                    route_source=self.source, obstacle_guard=self.avoidance is not None,
                    waypoint_acceptance_m=WAYPOINT_ACCEPTANCE_M)

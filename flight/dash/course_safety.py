"""Known-wall command checks. No ROS, arming, or inference of free space.

The depth source only reports wall-like surfaces. These checks cannot certify
unobserved space, overhead clearance, moving obstacles or aircraft dynamics.
Braking assumes 1.5 m/s² (half of PX4 MPC_ACC_HOR), not a measured stop.

A fresh, scaled frame with zero wall cells means "no wall seen in view": it is
valid input, not proof of free space. Stale, unscaled or missing frames are not
observations; ObstacleGuard bridges short gaps with its confirmed memory.
"""
import math

import numpy as np

MAX_DEPTH_AGE_S = 1.5
MAX_COURSE_SPEED_MPS = 1.0
TRACKING_RESERVE_M = .10
# Half of PX4 MPC_ACC_HOR (3.0 m/s^2 on this airframe) to allow for the jerk ramp.
# Calibration knob: raise toward MPC_ACC_HOR only after measuring real stops.
BRAKING_DECEL_MPS2 = 1.5
REACTION_S = .2
# Inside the clearance band a command may hold or move away, never closer.
INSIDE_SLACK_M = .02
# Closer than this to a wall cell the aircraft is treated as in contact: fail closed.
MIN_INSIDE_CLEARANCE_M = .10
# A cell is remembered once seen in two frames (same or neighbouring cell, VIO jitter).
CONFIRM_WITHIN_FRAMES = 4
# ...and forgotten only on positive evidence: well inside the camera view and absent (at any height)
# from FORGET_AFTER_MISSES consecutive frames. Real cells keep being seen; ML speckles do not.
FORGET_AFTER_MISSES = 5
FORGET_STOP_AFTER_MISSES = 10   # cells that stopped the aircraft need twice the evidence
# Hard stops: confirmed (2-frame) cells anywhere, but a cell seen in this frame only if it is this close
# (no time to wait for a second frame). Farther single-frame cells (ML speckles) trigger nudges, not stops.
EMERGENCY_M = 1.0
STOP_MEMORY_M = 1.5             # on a stop, remember the last frame's cells this close to the aircraft
FORGET_VIEW_DEG = 35.
FORGET_RANGE_M = (.4, 3.0)
FORGET_VFOV_DEG = 25.      # half vertical field of view: cells outside it at that range are not "seen absent"



def observations(depth):
    if not isinstance(depth, dict) or depth.get('status') != 'live':
        raise ValueError('fresh obstacle observations are required')
    age = depth.get('age_s')
    if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= MAX_DEPTH_AGE_S:
        raise ValueError('obstacle observations are stale or have no valid age')
    cell = depth.get('cell_m')
    if type(cell) not in (int, float) or not math.isfinite(cell) or not .02 <= cell <= .5:
        raise ValueError('invalid obstacle cell size')
    points = depth.get('points')
    if not isinstance(points, (list, tuple)) or len(points) > 1600:
        raise ValueError('obstacle observations must be a list of at most 1600 cells')
    if any(not isinstance(p, (list, tuple)) or len(p) != 2
           or any(type(v) not in (int, float) or not math.isfinite(v) for v in p) for p in points):
        raise ValueError('invalid obstacle coordinates')
    return np.asarray(points, dtype=float).reshape(-1, 2), float(cell), float(age)


def segment_clearance(start, end, walls, cell_m):
    walls = np.asarray(walls, float).reshape(-1, 2)
    if not len(walls):
        return math.inf
    a, b = np.asarray(start[:2], float), np.asarray(end[:2], float)
    delta, direction = walls-a, b-a
    fraction = np.clip(delta @ direction / max(float(direction @ direction), 1e-24), 0., 1.)
    return float(np.linalg.norm(delta-fraction[:, None]*direction, axis=1).min())-cell_m/math.sqrt(2)


def _check_sweeps(walls, cell, age, position, velocity, output, previous_command, dt, clearance_m,
                  route=None):
    values = (*position, *velocity, *previous_command, *output[1], dt, clearance_m, age)
    if not all(v is not None and math.isfinite(v) for v in values) or clearance_m < .15:
        raise ValueError('invalid obstacle guard position, velocity or clearance')
    p = np.asarray(position[:2], float)
    here = segment_clearance(p, p, walls, cell)
    if here < MIN_INSIDE_CLEARANCE_M:
        raise ValueError('aircraft is inside an obstacle cell clearance')
    # Outside the band every sweep keeps full clearance. Inside it (after a stop
    # near a wall) a sweep may hold or move away but never get closer.
    required = clearance_m if here >= clearance_m else here-INSIDE_SLACK_M
    measured = np.asarray(velocity[:2], float)
    if output[0] == 'pose':
        target = output[1]
        proposed = (np.asarray(target[:2])-np.asarray(previous_command[:2]))/max(dt, .001)
        if segment_clearance(p, target, walls, cell) < required:
            raise ValueError('commanded position crosses obstacle clearance')
    elif output[0] == 'twist':
        proposed = np.asarray(output[1][:2], float)
    else:
        raise ValueError('unknown command type')
    # With the planned route ahead, check that path rather than a straight line
    # along the commanded velocity: a course that bends away before a wall is
    # not heading into it. The measured-velocity sweep still catches drift.
    sweeps = (measured, proposed)
    if route is not None and len(route) >= 2:
        for a, b in zip(route, route[1:]):
            if segment_clearance(a, b, walls, cell) < required:
                raise ValueError('commanded position crosses obstacle clearance')
        sweeps = (measured,)
    # Without a route: reaction plus braking distance along each velocity. With
    # the route checked above, the measured-velocity sweep only covers the
    # reaction distance: it catches real drift into a wall, not walls the
    # planned arc already avoids. No frame-age term: cells are world-fixed.
    braking = route is None or len(route) < 2
    for v in sweeps:
        speed = float(np.linalg.norm(v))
        stop = p+v*(REACTION_S+(speed/(2*BRAKING_DECEL_MPS2) if braking else 0.))
        if segment_clearance(p, stop, walls, cell) < required:
            raise ValueError('obstacle inside the commanded or measured stopping path')


def check_command(depth, position, velocity, output, previous_command, dt, clearance_m, known=None):
    """Reject the measured-to-command sweep and both stopping-distance sweeps.

    Checking complete segments catches corner cutting and velocity corrections
    that would leave an otherwise clear polyline. A refusal is latched by the
    caller, which requests the interface's current-position hold.
    """
    walls, cell, age = observations(depth)
    if known is not None:
        walls, cell = known
    _check_sweeps(walls, cell, age, position, velocity, output, previous_command, dt, clearance_m)


ROUTE_STEP_S = .1


def route_ahead(run, position, speed_mps, margin_m=.30):
    """XY polyline from the aircraft along the run's planned route, out to the
    stopping distance at ``speed_mps`` plus ``margin_m``."""
    speed = max(float(speed_mps), 0.)
    reach = REACTION_S*speed + speed*speed/(2*BRAKING_DECEL_MPS2) + float(margin_m)
    points = [tuple(float(v) for v in position[:2])]
    t, end, total = float(run.elapsed), float(run.times[-1]), 0.
    while t < end and total < reach:
        t = min(end, t+ROUTE_STEP_S)
        q = tuple(float(v) for v in run.sample(t)[:2])
        total += math.dist(points[-1], q)
        points.append(q)
    return points


class ObstacleGuard:
    """Keep confirmed occupied cells for this flight, even after turning away.

    Disappearing detections never clear a confirmed cell. A cell seen in only
    one frame is still checked in that frame but is not remembered, so single
    ML-depth speckles cannot block a route forever. Bounded memory fails closed
    on overflow; a new home/reference starts a new map.
    """
    def __init__(self, config):
        self.clearance = config['clearance_m']
        self.cells = {}
        self.cell_m = .02
        self._seen = {}          # grid index -> frame number last seen (unconfirmed)
        self._frame = 0
        self._version = None
        self._last = None        # (walls, cell) of the last usable frame
        self.z = {}              # cell key -> list of (z_lo, z_hi) seen there (height-aware memory)
        self.misses = {}         # cell key -> consecutive in-view frames it was absent from
        self.stop_cells = set()  # cells that stopped the aircraft: forgotten only on stronger evidence
        self._last_t = None      # monotonic time and version of the last usable frame
        self._last_version = None
        self._rev = 0            # memory revision, for the height-mask cache
        self._mask_cache = None
        self._mask_route = None  # (id, len) of the route the per-cell masks below were computed for
        self._mask_keys = {}     # cell key -> kept at that route's height
        self._dirty = set()      # cells whose heights changed since
        if config.get('walls'):
            self._remember(config['walls'], config['cell_m'])

    def _remember(self, points, cell, zmap=None):
        self.cell_m = max(self.cell_m, cell)
        self._rev += 1
        for point in points:
            key = tuple(round(float(v), 3) for v in point)
            self.cells[key] = tuple(point)
            for zr in (zmap or {}).get(key, ()):
                zr = (round(float(zr[0]), 2), round(float(zr[1]), 2))      # quantised: bounded growth
                ranges = self.z.setdefault(key, [])
                if zr not in ranges and len(ranges) < 12:
                    ranges.append(zr)
                    self._dirty.add(key)
            if len(self.cells) > 6400:
                raise ValueError('obstacle memory is full; reset the course reference')

    def inherit(self, other):
        """Take over another guard's whole memory: cells, heights, pending (unconfirmed) sightings and
        the last frame, so an obstacle seen intermittently is still confirmed across a nudge."""
        self._remember(other.cells.values(), other.cell_m)
        for key, ranges in other.z.items():
            mine = self.z.setdefault(key, [])
            mine.extend(r for r in ranges if r not in mine)
        self._seen.update(other._seen)
        self._frame = max(self._frame, other._frame)
        self._version = other._version
        self.misses.update(other.misses)
        self.stop_cells |= other.stop_cells
        if other._last is not None and self._last is None:
            self._last, self._last_z = other._last, getattr(other, '_last_z', None)
            self._last_t, self._last_version = other._last_t, other._last_version

    def _memory(self, height_route=None, here=None):
        """Remembered XY cells; with height_route, only those whose height overlaps the route
        there (cells with no recorded height are always kept), plus any near the aircraft at its
        actual height (`here`, e.g. while off its planned height). The route mask is cached."""
        keys = list(self.cells)
        xy = np.asarray([self.cells[k] for k in keys], dtype=float).reshape(-1, 2)
        if height_route is None or not len(xy) or not self.z:
            return xy
        key = (self._rev, id(height_route), len(height_route))
        if self._mask_cache is not None and self._mask_cache[0] == key:
            base = self._mask_cache[1]
        else:
            base = self._height_mask(keys, xy, height_route)
            self._mask_cache = (key, base)
        if here is not None:
            from depth_height import DRONE_HALF_HEIGHT_M, HEIGHT_MARGIN_M
            band = DRONE_HALF_HEIGHT_M+HEIGHT_MARGIN_M
            near = np.flatnonzero(np.hypot(xy[:, 0]-here[0], xy[:, 1]-here[1]) <= 1.0)
            extra = [i for i in near if any(r[0] <= here[2]+band and r[1] >= here[2]-band
                                            for r in self.z.get(keys[i]) or [(-math.inf, math.inf)])]
            if extra:
                base = np.unique(np.r_[base, np.asarray(extra, dtype=int)])
        return xy[base] if len(base) else np.empty((0, 2))

    def _height_mask(self, keys, xy, height_route):
        """Indices of `keys` kept at the route's height; incremental per cell (new / changed cells only)."""
        rk = (id(height_route), len(height_route))
        if self._mask_route != rk:
            self._mask_route, self._mask_keys, self._dirty = rk, {}, set()
        todo = [i for i, k in enumerate(keys) if k not in self._mask_keys or k in self._dirty]
        if todo:
            res = self._height_mask_full([keys[i] for i in todo], xy[todo], height_route)
            for j, i in enumerate(todo):
                self._mask_keys[keys[i]] = bool(res[j])
            self._dirty -= {keys[i] for i in todo}
        return np.asarray([i for i, k in enumerate(keys) if self._mask_keys.get(k)], dtype=int)

    def _height_mask_full(self, keys, xy, height_route):
        """Per-cell bool: any of its height ranges overlaps the route's height band there."""
        from depth_height import keep_mask
        rows, zr = [], []
        for i, k in enumerate(keys):
            for r in self.z.get(k) or [(-math.inf, math.inf)]:
                rows.append(i)
                zr.append(r)
        rows = np.asarray(rows)
        mask = keep_mask(xy[rows], zr, height_route)
        out = np.zeros(len(keys), dtype=bool)
        out[rows[mask]] = True
        return out

    def route_conflict(self, points, clearance_m, height_route=None):
        """True if the planned polyline comes within clearance_m of a remembered
        cell or a cell in the last usable frame (used to trigger a nudge)."""
        walls = [self._memory(height_route)]
        if self._last is not None:
            walls.append(self._last[0])
        walls = np.vstack(walls)
        if not len(walls) or len(points) < 2:
            return False
        cell = max(self.cell_m, self._last[1] if self._last is not None else 0.)
        return any(segment_clearance(a, b, walls, cell) < clearance_m for a, b in zip(points, points[1:]))

    def conflict_cells(self, points, clearance_m, height_route=None):
        """The remembered / last-frame cells within clearance_m of the polyline (a nudge's trigger)."""
        walls = [self._memory(height_route)]
        if self._last is not None:
            walls.append(self._last[0])
        walls = np.vstack(walls)
        pts = np.asarray([p[:2] for p in points], dtype=float).reshape(-1, 2)
        if not len(walls) or len(pts) < 2:
            return []
        cell = max(self.cell_m, self._last[1] if self._last is not None else 0.)
        hit = np.zeros(len(walls), dtype=bool)
        for a, b in zip(pts, pts[1:]):
            d = b-a
            f = np.clip((walls-a) @ d / max(float(d @ d), 1e-24), 0., 1.)
            hit |= np.linalg.norm(walls-(a+f[:, None]*d), axis=1)-cell/math.sqrt(2) < clearance_m
        return walls[hit].tolist()

    def known_walls(self, height_route=None):
        """Remembered cells plus the last usable frame, for in-flight nudges."""
        cells = [tuple(p) for p in self._memory(height_route).tolist()]
        if self._last is not None:
            cells += [tuple(p) for p in self._last[0].tolist()]
        return cells

    def remember_last(self, near=None):
        """Confirm the last usable frame's cells (used when it stopped us): with `near` (x, y) only those
        within STOP_MEMORY_M of it, not every speckle in the frame."""
        if self._last is not None and len(self._last[0]):
            w = np.asarray(self._last[0], dtype=float).reshape(-1, 2)
            if near is not None:
                w = w[np.hypot(w[:, 0]-near[0], w[:, 1]-near[1]) <= STOP_MEMORY_M]
            pts = w.tolist()
            if not pts:
                return
            self._remember(pts, self._last[1], getattr(self, '_last_z', None))
            self.stop_cells |= {tuple(round(float(v), 3) for v in p) for p in pts}

    def _observe(self, walls, cell, version, zmap=None):
        if version is not None and version == self._version:
            return  # the same frame again is not a second sighting
        self._version = version
        self._frame += 1
        grid = {(int(math.floor(x/cell)), int(math.floor(y/cell))): (float(x), float(y))
                for x, y in walls.tolist()}
        confirmed = []
        for (i, j), point in grid.items():
            if tuple(round(v, 3) for v in point) in self.cells:
                self._remember([point], cell, zmap)   # record heights seen there later too
                continue
            if any(self._frame-self._seen.get((i+di, j+dj), -99) <= CONFIRM_WITHIN_FRAMES
                   for di in (-1, 0, 1) for dj in (-1, 0, 1)):
                confirmed.append(point)
        for key in grid:
            self._seen[key] = self._frame
        self._seen = {k: f for k, f in self._seen.items() if self._frame-f <= CONFIRM_WITHIN_FRAMES}
        if confirmed:
            self._remember(confirmed, cell, zmap)

    def forget_unseen(self, depth, camera):
        """camera = (x, y, yaw[, z]). Drop remembered cells the camera clearly looks at but no longer
        sees: inside the view cone and vertical field of view, not hidden behind a nearer observed cell,
        and never a cell that stopped the aircraft."""
        if camera is None or not self.cells:
            return 0
        seen = depth.get('cells3d') or [p[:2] for p in depth.get('points') or []]
        seen = np.asarray([c[:2] for c in seen], dtype=float).reshape(-1, 2)
        cx, cy, yaw = camera[:3]
        cz = camera[3] if len(camera) > 3 else None
        c, s = math.cos(yaw), math.sin(yaw)
        if len(seen):
            sd = np.hypot(seen[:, 0]-cx, seen[:, 1]-cy)
            sb = np.arctan2(seen[:, 1]-cy, seen[:, 0]-cx)
        dropped = 0
        for key, (x, y) in list(self.cells.items()):
            dx, dy = x-cx, y-cy
            dist = math.hypot(dx, dy)
            if not (FORGET_RANGE_M[0] <= dist <= FORGET_RANGE_M[1]
                    and math.degrees(abs(math.atan2(-s*dx+c*dy, c*dx+s*dy))) <= FORGET_VIEW_DEG):
                continue                       # not clearly in view: no evidence either way
            if cz is not None:
                band = dist*math.tan(math.radians(FORGET_VFOV_DEG))
                if not any(r[0] <= cz+band and r[1] >= cz-band for r in self.z.get(key) or [(-math.inf, math.inf)]):
                    continue                   # above/below the camera's view at this range
            if len(seen) and float(np.min(np.hypot(seen[:, 0]-x, seen[:, 1]-y))) <= .12:
                self.misses.pop(key, None)
                continue
            if len(seen):
                bearing = math.atan2(dy, dx)
                db = np.abs((sb-bearing+math.pi) % (2*math.pi)-math.pi)
                if np.any((db <= math.radians(4.)) & (sd < dist-.15)):
                    continue                   # hidden behind something nearer: no evidence
            self.misses[key] = self.misses.get(key, 0)+1
            if self.misses[key] >= (FORGET_STOP_AFTER_MISSES if key in self.stop_cells else FORGET_AFTER_MISSES):
                self.cells.pop(key, None); self.z.pop(key, None); self.misses.pop(key, None)
                self.stop_cells.discard(key)
                self._rev += 1
                dropped += 1
        return dropped

    def check(self, depth, position, velocity, output, previous_command, dt, now=None, route=None,
              height_route=None, camera=None, here=None):
        """Check one command. With ``now`` (monotonic s, i.e. in flight) a stale,
        unscaled or missing frame never stops the course by itself: commands are
        still checked against confirmed memory (no frame-age term). Operator choice: only an
        obstacle in the path stops the aircraft. Without ``now`` (start checks)
        a usable frame is still required."""
        try:
            walls, cell, age = observations(depth)
        except ValueError:
            if now is None:
                raise
            # Remembered cells are world-fixed and the pose is current, so no
            # frame-age term applies to them (it only covers motion since capture).
            walls, cell, age = np.empty((0, 2)), self.cell_m, 0.
        else:
            zmap = {}
            for c in depth.get('cells3d') or ():
                zmap.setdefault((round(float(c[0]), 3), round(float(c[1]), 3)), []).append(
                    (float(c[2]), float(c[3])))
            if depth.get('geometry_version') is None or depth.get('geometry_version') != self._version:
                self.forget_unseen(depth, camera)      # once per new frame
            self._observe(walls, cell, depth.get('geometry_version'), zmap)
            self._last, self._last_z = (walls, cell), zmap
            self._last_t, self._last_version = now, depth.get('geometry_version')
        memory = self._memory(height_route, here)
        if len(walls):                    # this frame's unconfirmed cells stop us only when this close
            walls = walls[np.hypot(walls[:, 0]-position[0], walls[:, 1]-position[1]) <= EMERGENCY_M]
        _check_sweeps(np.vstack([memory, walls]), max(cell, self.cell_m), age, position, velocity,
                      output, previous_command, dt, self.clearance, route=route)

"""Pass-through centring for openings (tunnels, windows). No ROS.

Only at holes the operator marked (Plan tab): with no marked hole nothing here ever acts.
Two states while a course flies:
  normal   - the route as planned (avoidance nudges as usual);
  passing  - the route ahead goes through an opening near a marked hole: depth cells
             enclose the drone there (both sides at its height, or above and below it).
             That stretch of the route is shifted sideways and up/down toward the middle
             of the free space, re-measured on every depth frame, blended in and out with
             smooth ramps and rate-limited. Back to normal after the exit.
A solid obstacle in the path (something you would go under or around) is not an opening.

Works on the route itself (CourseRun.points), not on the setpoint, so the guard,
the height filter and the lookahead all see where the aircraft will fly.
Cells are [x, y, z_lo, z_hi] in the route's frame (world ENU in flight).
"""
import math

import numpy as np

HALF_W_M = .26        # calibration knob: half the drone width the opening is threaded with (20 in)
HALF_H_M = .12        # half the drone height, props included
SIDE_M = .90          # walls this close on both sides (or above and below) make an opening
LOOK_M = 3.0          # route distance ahead searched for an opening
BIN_M = .10           # along-route resolution of the opening search
PAD_M = .30           # full shift this far before the entrance and after the exit
RAMP_M = 1.0          # blend from the planned route to the centred one over this distance
MAX_SHIFT_M = .40     # never move the route more than this from the plan
RATE_MPS = .30        # the shift changes at most this fast while flying
MIN_GAIN_M = .01      # centre only if it gains at least this much room
CONFIRM_SHIFT_M = .10 # two consecutive frames must agree on the shift this closely to engage
HOLE_SNAP_M = 1.0     # a marked hole this close to the route (XY) bends the route through it
HOLE_MAX_SHIFT_M = 1.0  # how far a marked hole may pull the route
HOLE_ZONE_M = 1.2     # around a marked hole: always centre on the measured opening (relaxed rule)
ROBUST_K = 1          # nearest cell decides, like the guard (k=2 let one cell sit inside the drone)


def _arc(points):
    p = np.asarray(points, dtype=float)
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1))]


def _normals(points):
    """Left normal of the XY tangent at each point (zero where the route is vertical)."""
    p = np.asarray(points, dtype=float)[:, :2]
    t = np.gradient(p, axis=0)
    n = np.linalg.norm(t, axis=1, keepdims=True)
    t = np.where(n > 1e-9, t/np.maximum(n, 1e-12), 0.)
    return t, np.stack([-t[:, 1], t[:, 0]], axis=1)


GATE_DEPTH_M = 1.0     # cells this far before/after the hole, along its axis, belong to its cross-section


def _route_frame(pts):
    T, N = _normals(pts)
    return _arc(pts), T, N


def _hole_passes(pts, s, hole, s_lo=-math.inf, s_hi=math.inf):
    """Route indices where the route passes the hole (each local XY minimum within HOLE_SNAP_M)."""
    d = np.linalg.norm(pts[:, :2]-np.asarray(hole[:2], float), axis=1)
    out, i = [], 0
    while i < len(d):
        if d[i] <= HOLE_SNAP_M and s_lo <= s[i] <= s_hi:
            j = i
            while j+1 < len(d) and d[j+1] <= HOLE_SNAP_M:
                j += 1
            k = i+int(np.argmin(d[i:j+1]))
            out.append(k)
            i = j+1
        else:
            i += 1
    return out


def _opening_at(pts, s, T, N, i_h, c, floor, ceiling):
    """Opening in the gate plane at route index i_h: cells projected on the route's axis there
    (so turns before/after the hole cannot misassign them)."""
    t, n, P = T[i_h], N[i_h], pts[i_h]
    if np.linalg.norm(t) == 0 or not len(c):
        return None
    rel = c[:, :2]-P[:2]
    fwd, lat = rel @ t, rel @ n
    near = (np.abs(fwd) <= GATE_DEPTH_M) & (np.abs(lat) <= SIDE_M)
    if not near.any():
        return None
    c, fwd, lat = c[near].copy(), fwd[near], lat[near]
    bad = ~np.isfinite(c[:, 2:]).all(axis=1)          # unknown height: spans all heights
    c[bad, 2], c[bad, 3] = -10., 10.
    zp = float(P[2])
    band = HALF_H_M + .15
    side = (c[:, 2] <= zp+band) & (c[:, 3] >= zp-band)
    keys = [(round(x, 3), round(y, 3)) for x, y in c[:, :2].tolist()]
    walled = {k for k, s_ in zip(keys, side) if s_}
    free_col = np.array([k not in walled for k in keys], dtype=bool)
    over = (np.abs(lat) <= HALF_W_M+.10) & free_col
    above, below = over & (c[:, 2] > zp+HALF_H_M), over & (c[:, 3] < zp-HALF_H_M)
    left, right = side & (lat > 0), side & (lat < 0)
    bins = np.floor(fwd/BIN_M).astype(int)
    present = set(bins.tolist())
    is_open = {b for b in present if ((left[bins == b].any() and right[bins == b].any())
                                     or (above[bins == b].any() and below[bins == b].any()))}
    if not is_open:
        return None
    # the contiguous run of opening bins nearest the hole (1-bin gaps allowed)
    start = min(is_open, key=lambda b: (abs(b+.5), b))
    first = last = start
    while any(b in is_open for b in (first-1, first-2)):
        first = max(b for b in (first-1, first-2) if b in is_open)
    while any(b in is_open for b in (last+1, last+2)):
        last = max(b for b in (last+1, last+2) if b in is_open)
    f0, f1 = first*BIN_M, (last+1)*BIN_M
    m = (fwd >= f0-BIN_M) & (fwd <= f1+BIN_M)
    ml, zlo, zhi = lat[m], c[m, 2]-zp, c[m, 3]-zp
    grid = np.arange(-MAX_SHIFT_M, MAX_SHIFT_M+1e-9, .02)
    dl, dzs = np.meshgrid(grid, grid, indexing='ij')
    dl, dzs = dl.ravel(), dzs.ravel()
    z_ok = (zp+dzs >= floor) & (zp+dzs <= ceiling)
    side_gap = np.abs(ml[None, :]-dl[:, None])-HALF_W_M
    vert_gap = np.maximum(zlo[None, :]-(dzs[:, None]+HALF_H_M), (dzs[:, None]-HALF_H_M)-zhi[None, :])
    margin = np.maximum(side_gap, vert_gap)

    def nearest_cell(values):
        return values.min(axis=1) if values.shape[1] else np.full(len(values), 1.)
    room = nearest_cell(margin)
    lat_room = nearest_cell(np.where(vert_gap < 0, side_gap, 1.))
    vert_room = nearest_cell(np.where(side_gap < 0, vert_gap, 1.))
    score = room + .25*(np.minimum(lat_room, 1.)+np.minimum(vert_room, 1.)) - .05*np.hypot(dl, dzs)
    score = np.where(z_ok, score, -np.inf)
    best = int(np.argmax(score))
    if not np.isfinite(score[best]) or room[best] < 0:
        return None                                    # the drone fits nowhere: not an opening
    sg, vg, bdl, bdz = side_gap[best], vert_gap[best], dl[best], dzs[best]
    level, abeam = vg < 0, sg < 0
    lat_closed = (level & (ml > bdl)).any() and (level & (ml < bdl)).any()
    vert_closed = (abeam & (vg >= 0) & (zlo > bdz)).any() and (abeam & (vg >= 0) & (zhi < bdz)).any()
    if not (lat_closed or vert_closed):
        return None                                    # an obstacle to go around/under, not a hole
    if abs(bdl) >= MAX_SHIFT_M-1e-9 and not lat_closed:
        return None                                    # pinned at the shift limit, one side unbounded
    if abs(bdz) >= MAX_SHIFT_M-1e-9 and not vert_closed:
        return None

    def nearest(values, mask):
        return round(float(values[mask].min()), 2) if mask.any() else None
    zc = c[:, 2]-zp
    fmid = .5*(f0+f1)
    centre = [float(P[0]+bdl*n[0]+fmid*t[0]), float(P[1]+bdl*n[1]+fmid*t[1]), zp+float(bdz)]
    out = dict(s0=float(s[i_h]+f0), s1=float(s[i_h]+f1), lat=float(bdl), dz=float(bdz), i_gate=int(i_h),
               room=float(room[best]), room_before=float(room[np.argmin(np.hypot(dl, dzs))]), centre=centre,
               left_m=nearest(lat, m & left), right_m=nearest(-lat, m & right),
               above_m=nearest(zc, m & above), below_m=nearest(zp-c[:, 3], m & below))
    out['needed'] = bool(out['room'] >= out['room_before']+MIN_GAIN_M)
    if not out['needed']:
        out.update(lat=0., dz=0.)
    return out


def find_opening(points, cells, s_from=0., look_m=LOOK_M, floor=-math.inf, ceiling=math.inf, holes=()):
    """The first marked hole ahead (s_from .. s_from+look_m) whose opening is seen, measured in that
    hole's gate plane. Returns dict(s0, s1, lat, dz, centre, i_gate, room, room_before, bounds...)
    or None. Nothing marked: never acts."""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    c = np.asarray(cells if cells is not None else [], dtype=float).reshape(-1, 4)
    if len(pts) < 2 or not len(c) or not holes:
        return None
    s, T, N = _route_frame(pts)
    passes = sorted(i for h in holes for i in _hole_passes(pts, s, h, s_from-PAD_M, s_from+look_m))
    for i_h in passes:
        o = _opening_at(pts, s, T, N, i_h, c, floor, ceiling)
        if o is not None:
            return o
    return None


def weights(s, s0, s1):
    """1 over [s0-PAD, s1+PAD], smooth (C1) ramps of RAMP_M to 0 either side."""
    s = np.asarray(s, dtype=float)
    a, b = s0-PAD_M, s1+PAD_M
    u = np.clip(np.maximum((a-s)/RAMP_M, (s-b)/RAMP_M), 0., 1.)
    return 1.-(3*u*u-2*u**3)


def shifted(base, s0, s1, lat, dz, floor=-math.inf, ceiling=math.inf):
    base = np.asarray(base, dtype=float).reshape(-1, 3)
    s = _arc(base)
    # Never move the route's ends (start hover / final point): taper to zero over RAMP_M.
    w = weights(s, s0, s1)*np.clip(s/RAMP_M, 0., 1.)*np.clip((s[-1]-s)/RAMP_M, 0., 1.)
    _, N = _normals(base)
    out = base.copy()
    out[:, :2] += (w*lat)[:, None]*N
    out[:, 2] = np.clip(base[:, 2]+w*dz, floor, ceiling)
    return out


def through_holes(points, holes, floor=-math.inf, ceiling=math.inf):
    """Bend the route through each marked hole centre it passes within HOLE_SNAP_M of."""
    route = np.asarray(points, dtype=float).reshape(-1, 3)
    used = []
    for h in holes or ():
        h = np.asarray(h, dtype=float)[:3]
        s = _arc(route)
        _, N = _normals(route)
        d = np.linalg.norm(route[:, :2]-h[:2], axis=1)
        i = int(d.argmin())
        if d[i] > HOLE_SNAP_M:
            continue
        lat = float(np.clip((h[:2]-route[i, :2]) @ N[i], -HOLE_MAX_SHIFT_M, HOLE_MAX_SHIFT_M))
        dz = float(np.clip(h[2]-route[i, 2], -HOLE_MAX_SHIFT_M, HOLE_MAX_SHIFT_M))
        route = shifted(route, s[i], s[i], lat, dz, floor, ceiling)
        used.append(dict(hole=h.tolist(), s=float(s[i]), lat=lat, dz=dz))
    return route, used


def centre_route(points, cells, floor=-math.inf, ceiling=math.inf, max_openings=3, holes=()):
    """Static centring (preview before takeoff): every opening the route crosses."""
    route = np.asarray(points, dtype=float).reshape(-1, 3)
    found, s_from = [], 0.
    for _ in range(max_openings):
        o = find_opening(route, cells, s_from, look_m=math.inf, floor=floor, ceiling=ceiling, holes=holes)
        if o is None:
            break
        if o['needed']:
            route = shifted(route, o['s0'], o['s1'], o['lat'], o['dz'], floor, ceiling)
            found.append(o)
        s_from = o['s1']+BIN_M
    return route, found


APPROACH_M = 1.5      # hover this far before a marked hole (along the route) to measure it
APPROACH_SLACK_M = .35  # ...starting no closer than APPROACH_M - this (else: too close to measure, stop)
LINEUP_M = .8         # line up on the gate axis up to this far before the centre...
EXIT_M = 1.0          # ...and fly straight through up to this far past it (blind: the gate leaves the view)
STRAIGHT_DEG = 20.    # lineup/exit legs never extend past where the plan turns by more than this
CONVERGE_N = 4        # commit once the last N frames all gave an estimate...
CONVERGE_M = .03      # ...all within this of their median (the estimate has stopped moving)...
MIN_HOVER_S = 1.5     # ...and the drone has hovered at least this long
HOVER_MAX_S = 15.     # no steady estimate by then: give up (the course stops, the operator decides)
COMMIT_PACE = .4      # fraction of course speed through the commit segment


class PassThrough:
    """Hole control for one CourseRun, at marked holes (Plan tab) inside an avoidance zone.

    normal     fly the route as planned (preview already bent it through the mark)
    hover      APPROACH_M before a hole: the course clock stops (`hold`); every depth frame measures the
               opening in the gate plane and gives a world-frame estimate of the gate centre
    commit     once CONVERGE_N consecutive estimates agree within CONVERGE_M: the route from here becomes
               a straight line, lined up on the gate axis, through the centre and out past it (legs capped
               to where the plan runs straight), then blended back onto the plan. No visual correction in
               this segment (the gate leaves the camera view up close); the guard still runs elsewhere.
    too_close  a hole ahead is nearer than the approach point (e.g. after a replan): stop, say so
    gave_up    no steady estimate within HOVER_MAX_S: stop, say so
    Never commits on an estimate that is still moving."""

    def __init__(self, points, holes=(), done=()):
        self.base = np.asarray(points, dtype=float).reshape(-1, 3)
        self.s = _arc(self.base)
        done = [list(d)[:3] for d in done or ()]
        self.holes = []
        for h in holes or ():
            h = [float(v) for v in list(h)[:3]]
            T, _ = _normals(self.base)
            for i in _hole_passes(self.base, self.s, h):
                if i == 0 and float((np.asarray(h[:2])-self.base[0, :2]) @ T[0]) < -.05:
                    continue                       # behind the run's start (e.g. a replan past it)
                self.holes.append(dict(at=h, s=float(self.s[i]), i=i,
                                       done=any(math.dist(h, d) < 1e-6 for d in done)))
        self.holes.sort(key=lambda x: x['s'])
        self.state, self.hold, self.active = 'normal', False, None
        self.estimates, self.hover_s, self.gate, self.exit_s = [], 0., None, None
        self.frame, self.seen, self.reason, self.i_now = None, {}, '', 0

    def s_at(self, xyz):
        """Progress along the route, searched forward from the last position (out-and-back safe)."""
        lo, hi = max(0, self.i_now-30), min(len(self.base), self.i_now+400)
        i = lo+int(np.linalg.norm(self.base[lo:hi, :2]-np.asarray(xyz[:2], float), axis=1).argmin())
        self.i_now = i
        return float(self.s[i])

    def done_holes(self):
        return [h['at'] for h in self.holes if h['done'] or (self.state == 'commit' and h is self._active())]

    def _active(self):
        return None if self.active is None else self.holes[self.active]

    def _cells(self, cells, here):
        """Accumulate cells over frames (walls do not move; the view is forward only)."""
        for cell in cells:
            k = (round(cell[0], 2), round(cell[1], 2), round(cell[2], 1), round(cell[3], 1))
            e = self.seen.get(k)
            if e is None:
                self.seen[k] = [list(cell), 1]
            else:
                e[1] += 1
        hx, hy = float(here[0]), float(here[1])
        self.seen = {k: e for k, e in self.seen.items() if math.hypot(k[0]-hx, k[1]-hy) <= 4.}
        if len(self.seen) > 6000:            # ponytail: bounded; keep the most-seen
            self.seen = dict(sorted(self.seen.items(), key=lambda kv: -kv[1][1])[:6000])
        current = {(round(c[0], 2), round(c[1], 2), round(c[2], 1), round(c[3], 1)) for c in cells}
        return [e[0] for k, e in self.seen.items() if e[1] >= 2 or k in current]

    def _straight(self, i, step):
        """Distance from index i along the plan (step=+1 ahead, -1 behind) before it turns > STRAIGHT_DEG."""
        T, _ = _normals(self.base)
        t0, d, j = T[i], 0., i
        while 0 <= j+step < len(self.base):
            if np.linalg.norm(T[j+step]) > 0 and float(T[j+step] @ t0) < math.cos(math.radians(STRAIGHT_DEG)):
                break
            d += float(np.linalg.norm(self.base[j+step, :2]-self.base[j, :2]))
            j += step
        return d

    def _commit_route(self, here, gate, i_gate, floor, ceiling):
        """here -> lineup -> gate centre -> exit (legs capped to the plan's straight stretches and the route
        end), then blend back; same samples and times as before; the final route point never moves."""
        T, _ = _normals(self.base)
        d = T[i_gate] if np.linalg.norm(T[i_gate]) > 0 else np.array([1., 0.])
        d3 = np.array([d[0], d[1], 0.])
        s_here, s_gate, s_end = self.s_at(here), float(self.s[i_gate]), float(self.s[-1])
        lineup_m = max(.3, min(LINEUP_M, self._straight(i_gate, -1), s_gate-s_here))
        tail = s_end-s_gate
        exit_m = min(EXIT_M, self._straight(i_gate, +1), max(tail-.3, .2))
        lineup, exit_ = gate-lineup_m*d3, gate+exit_m*d3
        s_exit = min(s_gate+exit_m, s_end)
        route = self.base.copy()
        seg = np.flatnonzero((self.s >= s_here) & (self.s <= s_exit))
        seg = seg[seg < len(route)-1]                       # never the final point
        if len(seg) < 2:
            return None, None
        line = np.array([np.asarray(here, float)[:3], lineup, gate, exit_])
        L = np.r_[0., np.cumsum(np.linalg.norm(np.diff(line, axis=0), axis=1))]
        f = (self.s[seg]-s_here)/max(s_exit-s_here, 1e-9)
        for k, j in enumerate(seg):
            x = f[k]*L[-1]
            m = min(int(np.searchsorted(L, x, side='right'))-1, len(L)-2)
            u = (x-L[m])/max(L[m+1]-L[m], 1e-9)
            route[j] = line[m]+u*(line[m+1]-line[m])
        off = exit_-self.base[seg[-1]]
        ramp = min(RAMP_M, max(s_end-s_exit, 1e-6))
        after = np.flatnonzero((self.s > self.s[seg[-1]]) & (self.s <= self.s[seg[-1]]+ramp))
        u = np.clip((self.s[after]-self.s[seg[-1]])/ramp, 0., 1.)
        route[after] += ((1.-(3*u*u-2*u**3))[:, None])*off
        route[-1] = self.base[-1]
        route[:, 2] = np.clip(route[:, 2], floor, ceiling)
        route[0], route[-1] = self.base[0], self.base[-1]
        return route, float(self.s[seg[-1]])

    def _next_hole(self, s_now):
        for k, h in enumerate(self.holes):
            if not h['done'] and k != self.active and h['s'] >= s_now-.05:
                return k, h
        return None, None

    def _arm(self, s_now):
        """Enter hover for the next hole once it is APPROACH_M ahead; refuse if already too close."""
        k, h = self._next_hole(s_now)
        if h is None or h['s']-s_now > APPROACH_M+.05:
            return False
        self.active, self.estimates, self.hover_s, self.gate = k, [], 0., None
        if h['s']-s_now < APPROACH_M-APPROACH_SLACK_M:
            self.state, self.hold = 'too_close', True
            self.reason = (f'Hole {h["s"]-s_now:.2f} m ahead: too close to measure it '
                           f'(needs {APPROACH_M:.1f} m); stopping')
            return True
        self.state, self.hold, self.reason = 'hover', True, 'Hovering to measure the hole'
        return True

    def update(self, here, cells, frame_id, dt, floor=-math.inf, ceiling=math.inf):
        """Advance one tick. Returns the route to fly (Nx3) if it changed, else None. `hold` says
        whether the course clock must stop (hovering at a hole)."""
        s_now = self.s_at(here)
        fresh = cells is not None and frame_id != self.frame
        if fresh:
            self.frame = frame_id
            use = self._cells(cells, here)
        if self.state in ('too_close', 'gave_up'):
            return None                                   # terminal: the core stops the course
        if self.state == 'normal':
            self._arm(s_now)
            return None
        h = self.holes[self.active]
        if self.state == 'hover':
            self.hold = True
            self.hover_s += max(dt, 0.)
            if fresh:
                o = find_opening(self.base, use, s_now, look_m=APPROACH_M+GATE_DEPTH_M+.5, floor=floor,
                                 ceiling=ceiling, holes=[h['at']])
                self.estimates = (self.estimates+[None if o is None else (np.asarray(o['centre']), o['i_gate'])])[-8:]
            recent = self.estimates[-CONVERGE_N:]
            if len(recent) == CONVERGE_N and all(e is not None for e in recent):
                pts = np.array([e[0] for e in recent])
                med = np.median(pts, axis=0)
                spread = float(np.max(np.linalg.norm(pts-med, axis=1)))
                self.reason = f'Hovering: hole estimate spread {spread*100:.0f} cm (commit at {CONVERGE_M*100:.0f})'
                if spread <= CONVERGE_M and self.hover_s >= MIN_HOVER_S:
                    route, s_exit = self._commit_route(here, med, recent[-1][1], floor, ceiling)
                    if route is not None:
                        self.gate, self.exit_s = med, s_exit
                        self.state, self.hold = 'commit', False
                        self.reason = 'Hole locked: lined up, flying straight through'
                        self.base, self.s = route, _arc(route)
                        return route
                    self.reason = 'Hole locked but no room left on the route to line up'
            else:
                seen = sum(e is not None for e in recent)
                self.reason = (f'Hovering: {seen}/{CONVERGE_N} consecutive hole estimates'
                               if any(e is not None for e in self.estimates) else 'Hovering: waiting to see the hole')
            if self.hover_s >= HOVER_MAX_S:
                self.state = 'gave_up'
                self.reason = f'No steady hole estimate after {HOVER_MAX_S:.0f} s ({self.reason}); stopping'
            return None
        # commit
        self.hold = False
        k, nxt = self._next_hole(s_now)
        if nxt is not None and nxt['s']-s_now <= APPROACH_M+.05:
            h['done'] = True                              # a second hole right after this one
            self._arm(s_now)
            return None
        if s_now >= min(self.exit_s+RAMP_M, float(self.s[-1])-.1):
            h['done'] = True
            self.state, self.active, self.reason = 'normal', None, 'Through the hole; back on the plan'
        return None

    def status(self):
        g = self.gate
        return dict(state=self.state, hold=self.hold, reason=self.reason,
                    estimates=sum(e is not None for e in self.estimates), hover_s=round(self.hover_s, 1),
                    gate=None if g is None else [round(float(v), 3) for v in g],
                    holes_left=sum(1 for h in self.holes if not h['done']))


if __name__ == '__main__':
    # Self-check: 80 cm square window across a straight route at y=3, opening centred
    # at x=+0.15, z=1.10; route at x=0, z=1.0 -> shift +0.15 left (x), +0.10 up.
    route = [[0., y/10, 1.] for y in range(61)]
    cells = []
    for xi in range(-12, 13):
        x = round(.15+xi*.08, 3)
        for zlo in np.arange(.4, 1.8, .2):
            zhi = zlo+.2
            inside = abs(x-.15) < .40 and zhi > .70 and zlo < 1.50
            if not inside:
                cells.append([x, 3.0, zlo, zhi])
    hole = [[.15, 3.0, 1.1]]
    assert find_opening(route, cells) is None                     # nothing marked: never acts
    o = find_opening(route, cells, holes=hole)
    assert o is not None and 2.9 <= o['s0'] <= 3.0 <= o['s1'] <= 3.2, o
    assert abs(o['lat']+.15) < .06 or abs(o['lat']-.15) < .06, o   # x+ is right of a +y route
    assert abs(o['dz']-.10) < .03 and o['room'] > o['room_before'], o
    r, found = centre_route(route, cells, holes=hole)
    mid = r[30]
    assert abs(mid[0]-.15) < .03 and abs(mid[2]-1.1) < .03, mid
    assert np.allclose(r[0], route[0]) and np.allclose(r[-1], route[-1])
    pt = PassThrough(route, holes=hole)
    assert pt.holes and abs(pt.holes[0]['s']-3.0) < .06
    pt.update([0., 1.0, 1.], cells, 'a', .1)
    assert pt.state == 'normal'                               # still 2 m away
    pt.update([0., 1.55, 1.], cells, 'b', .1)
    assert pt.state == 'hover' and pt.hold
    for k in range(30):                                       # hover: same view every frame -> converges
        r = pt.update([0., 1.55, 1.], cells, f'f{k}', .1)
        if pt.state == 'commit':
            break
    assert pt.state == 'commit' and not pt.hold, pt.status()
    g = pt.gate
    assert abs(g[0]-.15) < .03 and abs(g[2]-1.1) < .03, g    # locked on the measured centre
    near = min(r, key=lambda p: abs(p[1]-3.0))
    assert abs(near[0]-.15) < .03 and abs(near[2]-1.1) < .03, near
    assert np.allclose(r[0], route[0]) and np.allclose(r[-1], route[-1])
    print('pass_through self-check ok', o)

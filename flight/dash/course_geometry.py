"""Bounded course-frame transforms and wall-aware trajectory geometry.

Only supplied wall cells are checked. Unknown/unobserved space is not proven
free. Flight callers must check health, flight limits and outgoing commands.
Original routes are not edited.
The added displacement has C2 joins; an input polyline itself need not be smooth.
"""
import math
import time

import numpy as np


MAX_SAMPLES = 4000
MAX_WALLS = 1600
SPACING_M = .05
NUDGE_STEP_M = .05   # detour offset search resolution: smallest offset that clears
NUDGE_MARGIN_M = .05 # plus this much, when it fits, for tracking error
NUDGE_COARSE_M = .25 # coarse pass of the coarse-to-fine offset search


class _WorkLimit(Exception):
    pass


def _check_work(deadline):
    if time.perf_counter() >= deadline:
        raise _WorkLimit


def _xyz(points):
    p = np.asarray(points, dtype=float)
    if p.ndim not in (1, 2) or p.shape[-1:] != (3,) or not np.isfinite(p).all():
        raise ValueError('points must be finite XYZ coordinates')
    return p


def _anchor(anchor):
    try:
        origin = np.asarray(anchor['origin_enu'], dtype=float)
        yaw = float(anchor['yaw_rad'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('anchor requires origin_enu and yaw_rad') from exc
    if origin.shape != (3,) or not np.isfinite(origin).all() or not math.isfinite(yaw):
        raise ValueError('anchor must be finite')
    c, s = math.cos(yaw), math.sin(yaw)
    return origin, np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def course_to_world(points_xyz, anchor):
    """Forward/left/up course coordinates -> PX4 local ENU; preserve input shape."""
    origin, rotation = _anchor(anchor)
    return _xyz(points_xyz) @ rotation.T + origin


def world_to_course(points_xyz, anchor):
    """PX4 local ENU -> forward/left/up course coordinates; preserve input shape."""
    origin, rotation = _anchor(anchor)
    return (_xyz(points_xyz)-origin) @ rotation


def bump(u):
    """Unit C2 bump: zero value, first and second derivative at u=0 and u=1."""
    x = np.asarray(u, dtype=float)
    if not np.isfinite(x).all():
        raise ValueError('bump parameter must be finite')
    x = np.clip(x, 0., 1.)
    return 64.*x**3*(1.-x)**3


def _resample(points, deadline):
    """Preserve vertices and linearly split segments; check bounds before allocating."""
    _check_work(deadline)
    if points.ndim != 2 or len(points) < 2 or len(points) > MAX_SAMPLES:
        raise ValueError('route requires 2–4000 XYZ points')
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if not np.isfinite(lengths).all() or (lengths > MAX_SAMPLES*SPACING_M).any():
        raise ValueError('route exceeds resampling bound')
    counts = np.maximum(1, np.ceil(lengths/SPACING_M-1e-10).astype(int))
    if int(counts.sum())+1 > MAX_SAMPLES:
        raise ValueError('route would exceed 4000 samples at 5 cm spacing')
    pieces = []
    for i,(a,b,n) in enumerate(zip(points[:-1], points[1:], counts)):
        if i % 64 == 0:
            _check_work(deadline)
        pieces.append(a+(b-a)*np.arange(n)[:,None]/n)
    return np.concatenate([*pieces, points[-1:]], axis=0)


def _near_box(points_xy, walls_xy, distance):
    """Discard cells whose lower-bound distance to a segment block is too large."""
    lo, hi = np.min(points_xy,axis=0), np.max(points_xy,axis=0)
    gap = np.maximum(np.maximum(lo-walls_xy,walls_xy-hi),0.)
    return walls_xy[np.sum(gap*gap,axis=1) <= distance*distance+1e-12]


def _segment_distances(points_xy, walls_xy, deadline, max_distance=None):
    """Minimum distance from each complete segment to wall cell centres.

Chunking bounds temporary arrays even at maximum route and wall counts.
Degenerate XY segments (e.g. vertical motion) become point-distance checks.
"""
    _check_work(deadline)
    a, vector = points_xy[:-1], np.diff(points_xy, axis=0)
    out = np.full(len(a), np.inf)
    if not len(walls_xy):
        return out
    for begin in range(0, len(a), 64):
        _check_work(deadline)
        start, v = a[begin:begin+64], vector[begin:begin+64]
        nearby = (walls_xy if max_distance is None else
                  _near_box(points_xy[begin:begin+65],walls_xy,max_distance))
        if not len(nearby):
            continue
        delta = nearby[None,:,:]-start[:,None,:]
        denom = np.sum(v*v, axis=1)
        fraction = np.einsum('nmk,nk->nm', delta, v)/np.maximum(denom[:,None], 1e-24)
        fraction = np.clip(fraction, 0., 1.)
        distance = np.linalg.norm(delta-fraction[:,:,None]*v[:,None,:], axis=2)
        out[begin:begin+64] = np.min(distance, axis=1)
    _check_work(deadline)
    return out


def _minimum_distance(points_xy, walls_xy, deadline):
    """Exact global minimum, pruning wall cells using an existing upper bound."""
    _check_work(deadline)
    best = float(np.linalg.norm(walls_xy-points_xy[0],axis=1).min())
    for begin in range(0,len(points_xy)-1,64):
        _check_work(deadline)
        points = points_xy[begin:begin+65]
        nearby = _near_box(points,walls_xy,best)
        if len(nearby):
            best = min(best,float(_segment_distances(points,nearby,deadline).min()))
    _check_work(deadline)
    return best


def deform_preview(points_xyz, walls_xy, *, clearance_m=.30, max_offset_m=.60,
                   cell_m=.08, max_work_s=.40, pass_clearance_m=None):
    """Try bounded lateral bumps, preserving endpoint positions and baseline Z.

At most six conflict windows, three window sizes and twelve signed amplitudes
per window are considered. Collision checks include complete line segments,
not just vertices. Wall cells are conservatively bounded by circumscribed
circles, so reported clearance is centre distance minus half-cell diagonal.
    Blocked previews are never represented as a successful original-route fallback,
    except: with pass_clearance_m (the flight guard's clearance), an original route
    that already keeps that clearance from every cell is returned 'unchanged', so
    a gap the guard accepts (e.g. a window) is flown straight through.
    A work-budget expiry returns no preview points, including when the deadline
    expires during final validation. Checks between 64-segment chunks bound
    normal overshoot to one small NumPy operation batch (not a real-time guarantee).
"""
    if not math.isfinite(max_work_s) or max_work_s <= 0:
        raise ValueError('work budget must be finite and positive')
    deadline = time.perf_counter()+max_work_s
    try:
        return _deform_preview(points_xyz,walls_xy,clearance_m,max_offset_m,cell_m,deadline,
                               pass_clearance_m)
    except _WorkLimit:
        return {'status':'blocked', 'points':[], 'min_clearance_m':None,
                'reason':'Preview work limit reached before final wall validation.'}


def _deform_preview(points_xyz,walls_xy,clearance_m,max_offset_m,cell_m,deadline,pass_clearance_m=None):
    _check_work(deadline)
    original = _xyz(points_xyz)
    base = _resample(original,deadline)
    walls = np.asarray(walls_xy, dtype=float)
    if walls.size == 0:
        walls = np.empty((0,2))
    if walls.ndim != 2 or walls.shape[1] != 2 or len(walls) > MAX_WALLS or not np.isfinite(walls).all():
        raise ValueError('walls require at most 1600 finite XY cells')
    if (not all(math.isfinite(v) for v in (clearance_m, max_offset_m, cell_m))
            or clearance_m < 0 or not 0 <= max_offset_m <= 2 or cell_m <= 0):
        raise ValueError('invalid clearance, offset (0–2 m), or cell size')
    cell_radius = cell_m/math.sqrt(2)
    radius = clearance_m+cell_radius
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(base,axis=0),axis=1))]
    current = base.copy()
    distances = _segment_distances(current[:,:2],walls,deadline,radius)

    def finish(status, reason):
        # Re-splitting cannot change this polyline's geometry or its altitude profile.
        try:
            result = _resample(current,deadline)
        except ValueError:
            return {'status':'blocked', 'points':[],
                    'reason':'Adjusted route exceeds the 4000-sample bound.',
                    'min_clearance_m':None}
        minimum = (_minimum_distance(result[:,:2],walls,deadline)-cell_radius
                   if len(walls) else None)
        if status != 'blocked' and minimum is not None and minimum < clearance_m-1e-9:
            status, reason = 'blocked', 'Final segment validation found a wall conflict.'
        points = result.tolist() if status != 'blocked' else []
        _check_work(deadline)
        return {'status':status, 'points':points, 'reason':reason, 'min_clearance_m':minimum}

    def blocked(reason):
        # Fly-through rule: no room for the planning reserve, but the original
        # route keeps the flight guard's clearance from every cell (a window).
        if pass_clearance_m is not None and len(walls):
            minimum = _minimum_distance(base[:,:2],walls,deadline)-cell_radius
            if minimum >= pass_clearance_m-1e-9:
                _check_work(deadline)
                return {'status':'unchanged', 'points':base.tolist(), 'min_clearance_m':minimum,
                        'reason':f'Original route passes the opening at {minimum:.2f} m (flight-guard clearance)'}
        return finish('blocked', reason)

    if not np.any(distances < radius-1e-9):
        return finish('unchanged', 'No conflict with supplied wall cells; unobserved space is not assessed.')
    endpoint_distances = np.linalg.norm(current[[0,-1],None,:2]-walls[None,:,:], axis=2)
    if (endpoint_distances < radius-1e-9).any():
        return blocked('A preserved route endpoint is inside the wall clearance.')
    if max_offset_m == 0:
        return blocked('Wall conflict and no lateral adjustment allowance.')

    for _ in range(6):
        _check_work(deadline)
        conflicts = np.flatnonzero(distances < radius-1e-9)
        if not len(conflicts):
            return finish('adjusted', 'Bounded obstacle detour; unobserved space is not assessed.')
        first = int(conflicts[0])
        last = first
        while last+1 < len(distances) and distances[last+1] < radius-1e-9:
            last += 1
        centre = .5*(s[first]+s[last+1])
        half = max(.6, .5*(s[last+1]-s[first])+.35)
        # Nudge, don't leap: the smallest offset (5 cm steps) that clears, and at
        # that offset the widest window first (gentlest slope and curvature);
        # of the two sides, the one with more room.
        shapes = []
        windows = set()
        for extra in (1.2, .7, .35, 0.):
            _check_work(deadline)
            lo = max(0, int(np.searchsorted(s, centre-half-extra, side='right'))-1)
            hi = min(len(s)-1, int(np.searchsorted(s, centre+half+extra, side='left')))
            if (lo,hi) in windows or hi-lo < 2 or s[hi]-s[lo] < 1e-9:
                continue
            windows.add((lo,hi))
            direction = base[hi,:2]-base[lo,:2]
            if np.linalg.norm(direction) < 1e-9:
                direction = base[first+1,:2]-base[first,:2]
            length = np.linalg.norm(direction)
            if length < 1e-9:
                continue
            normal = np.array([-direction[1],direction[0]])/length
            shapes.append((lo, hi, normal, bump((s[lo:hi+1]-s[lo])/(s[hi]-s[lo]))))
        def try_offset(amplitude):
            """Widest window that clears at this offset; of its two sides, more room."""
            for lo, hi, normal, weights in shapes:
                found = None
                for sign in (1., -1.):
                    _check_work(deadline)
                    candidate = current[lo:hi+1].copy()
                    candidate[:,:2] += (sign*amplitude*weights)[:,None]*normal
                    deviations = np.linalg.norm(candidate[:,:2]-base[lo:hi+1,:2],axis=1)
                    if np.any(deviations > max_offset_m+1e-9):
                        continue
                    trial = _segment_distances(candidate[:,:2],walls,deadline,radius)
                    if np.any(trial < radius-1e-9):
                        continue
                    room = float(trial.min())
                    if found is None or room > found[0]:
                        found = (room,lo,hi,candidate,trial)
                if found is not None:
                    return found
            return None

        # Coarse-to-fine: 25 cm steps find roughly what is needed, then 5 cm steps
        # just below it find the smallest offset that clears (~11 sizes, not 30).
        best, below = None, 0.
        for amplitude in list(np.arange(NUDGE_COARSE_M, max_offset_m+1e-9, NUDGE_COARSE_M)) + [max_offset_m]:
            amplitude = min(float(amplitude), max_offset_m)
            best = try_offset(amplitude)
            if best is not None:
                for fine in np.arange(below+NUDGE_STEP_M, amplitude-1e-9, NUDGE_STEP_M):
                    finer = try_offset(float(fine))
                    if finer is not None:
                        best = finer
                        break
                break
            below = amplitude
        if best is None:
            return blocked('No bounded lateral bump clears the supplied wall cells.')
        _,lo,hi,candidate,trial = best
        # One extra step of margin for tracking error and position noise, if it fits.
        offset = candidate[:,:2]-base[lo:hi+1,:2]
        size = float(np.linalg.norm(offset, axis=1).max())
        if size > 1e-9 and size+NUDGE_MARGIN_M <= max_offset_m+1e-9:
            wider = current[lo:hi+1].copy()
            wider[:,:2] = base[lo:hi+1,:2]+offset*(size+NUDGE_MARGIN_M)/size
            wider_trial = _segment_distances(wider[:,:2],walls,deadline,radius)
            if not np.any(wider_trial < radius-1e-9):
                candidate, trial = wider, wider_trial
        current[lo:hi+1] = candidate
        distances[lo:hi] = trial
    if not np.any(distances < radius-1e-9):
        return finish('adjusted', 'Bounded obstacle detour; unobserved space is not assessed.')
    return blocked('Conflict-window budget exhausted; route remains blocked.')

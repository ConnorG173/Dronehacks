"""Bounded replanning of a paused course suffix; no ROS or flight commands.

The caller snapshots a stopped run, computes here outside the flight lock, and
checks its generation, reference, settled position and current sensing before
committing the returned path. Only supplied occupied cells are assessed;
disappearing detections never erase remembered occupancy.
"""
from bisect import bisect_right
import copy
import hashlib
import json
import math
from numbers import Real
import time

import numpy as np

from course_geometry import MAX_SAMPLES, MAX_WALLS, deform_preview
from course_safety import (MAX_DEPTH_AGE_S, TRACKING_RESERVE_M, INSIDE_SLACK_M,
                           MIN_INSIDE_CLEARANCE_M, observations, segment_clearance)
from path_heading import tangent_yaws


MAX_MEMORY_WALLS = 6400
MAX_DISPLAY_POINTS = 600
BACK_OFF_MAX_M = 1.5
BACK_OFF_MARGIN_M = .30   # room past the clearance edge so the detour can curve away
BACK_OFF_MARGINS_M = (.30, .60, 1.0)
BACK_OFF_STEP_M = .05


def _back_off(current, walls, cell, need_centre_m, margin_m=BACK_OFF_MARGIN_M):
    """Shortest straight leg from the hover to a point ``margin_m`` beyond the
    planned clearance. The leg is checked as a full segment and may never come
    closer to any cell than the hover already is."""
    p = np.asarray(current[:2], dtype=float)
    distances = np.linalg.norm(walls-p, axis=1)
    here = float(distances.min())-cell/math.sqrt(2)
    if here < MIN_INSIDE_CLEARANCE_M:
        raise ValueError('hover is inside an obstacle cell clearance; no back-off is checked')
    away = p-walls[int(distances.argmin())]
    away /= float(np.linalg.norm(away))
    best = None
    for angle in (0., .5, -.5, 1., -1., 1.5, -1.5):
        c, s = math.cos(angle), math.sin(angle)
        u = np.array([c*away[0]-s*away[1], s*away[0]+c*away[1]])
        for k in range(1, int(round(BACK_OFF_MAX_M/BACK_OFF_STEP_M))+1):
            q = p+u*BACK_OFF_STEP_M*k
            if float(np.linalg.norm(walls-q, axis=1).min()) >= need_centre_m+margin_m:
                if (segment_clearance(p, q, walls, cell) >= here-INSIDE_SLACK_M
                        and (best is None or k < best[0])):
                    best = (k, q)
                break
    if best is None:
        raise ValueError('no checked back-off leaves the wall clearance')
    return [float(best[1][0]), float(best[1][1]), float(current[2])]


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(name + ' must be finite')
    try:
        value = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError(name + ' must be finite') from exc
    if not math.isfinite(value):
        raise ValueError(name + ' must be finite')
    return value


def _point(value, dimension, name):
    try:
        if len(value) != dimension:
            raise ValueError(name + ' has the wrong dimensions')
        return [_number(v, name) for v in value]
    except TypeError as exc:
        raise ValueError(name + ' must be coordinates') from exc


def plan_remaining(points, times_s, elapsed_s, current_position, anchor,
                   reference, radius, depth, known_walls=(), *,
                   known_cell_m=.08, max_offset_m=1.5, clearance_m=.3,
                   floor=None, ceiling=None, max_work_s=.40):
    """Return a complete guarded path from a hover through the remaining suffix.

    ``points``, ``times_s`` and ``elapsed_s`` are copied from the paused run;
    all positions and walls are world ENU. The suffix starts at the next time
    endpoint, never by a nearest-point search that could skip a future loop.
    Only leading vertices exactly equal to the current hover are removed.

    The result has the CourseRun path schema, new speed-derived timing, the
    unchanged anchor/reference, full occupied-cell memory and display geometry.
    ``next_index`` identifies the first uncompleted input endpoint. Supply the
    paused run's floor/ceiling to validate its height envelope here as well as
    when constructing the replacement CourseRun. Any failure raises ValueError;
    no unchecked or original-route fallback is returned.
    """
    started = time.perf_counter()
    budget = _number(max_work_s, 'work budget')
    if budget <= 0:
        raise ValueError('work budget must be positive')
    deadline = started + budget

    def remaining_work():
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            raise ValueError('Replan work limit reached before final validation')
        return remaining

    try:
        walls, current_cell, age = observations(depth)
    except ValueError:
        # Depth dropout: plan from remembered cells alone, but never from nothing.
        if not len(known_walls):
            raise
        walls, current_cell, age = np.empty((0, 2)), float(known_cell_m), 0.
    radius = _number(radius, 'course radius')
    offset = _number(max_offset_m, 'maximum offset')
    clearance = _number(clearance_m, 'clearance')
    old_cell = _number(known_cell_m, 'remembered cell size')
    if not .5 <= radius <= 20:
        raise ValueError('course radius must be 0.5–20 m')
    if not 0 <= offset <= 2 or not .15 <= clearance <= 1:
        raise ValueError('invalid clearance or maximum offset')
    if not .02 <= old_cell <= .5:
        raise ValueError('invalid remembered cell size')
    low = -math.inf if floor is None else _number(floor, 'floor')
    high = math.inf if ceiling is None else _number(ceiling, 'ceiling')
    if low >= high:
        raise ValueError('invalid height bounds')
    if not isinstance(anchor, dict) or anchor.get('valid') is False or reference is None:
        raise ValueError('a valid course anchor and reference are required')
    origin = _point(anchor.get('origin_enu'), 3, 'anchor origin')
    _number(anchor.get('yaw_rad'), 'anchor yaw')
    current = _point(current_position, 3, 'current position')
    try:
        count = len(points)
        if not 2 <= count <= MAX_SAMPLES or len(times_s) != count:
            raise ValueError('course points and times must match within the sample bound')
        route = [_point(p, 3, 'course point') for p in points]
        times = [_number(t, 'course time') for t in times_s]
    except TypeError as exc:
        raise ValueError('course points and times must be finite sequences') from exc
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('course times must strictly increase')
    elapsed = _number(elapsed_s, 'elapsed course time')
    if elapsed < times[0] or elapsed >= times[-1]:
        raise ValueError('no remaining course interval at this elapsed time')
    next_index = bisect_right(times, elapsed)
    suffix = route[next_index:]
    while suffix and math.dist(current, suffix[0]) <= 1e-9:
        suffix.pop(0)
    if not suffix:
        raise ValueError('remaining course endpoint is already reached')
    baseline = [current] + suffix

    def check_bounds(candidate):
        if any(math.hypot(p[0]-origin[0], p[1]-origin[1]) > radius+1e-9
               or not low-1e-9 <= p[2] <= high+1e-9 for p in candidate):
            raise ValueError('remaining route is outside the course radius or height bounds')

    check_bounds(baseline)
    remaining_work()
    memory = {}
    try:
        for i, point in enumerate(known_walls):
            if i >= MAX_MEMORY_WALLS:
                raise ValueError('remembered obstacle input exceeds the memory bound')
            xy = _point(point, 2, 'remembered wall')
            memory[tuple(xy)] = xy
    except TypeError as exc:
        raise ValueError('remembered obstacles must be an iterable of XY cells') from exc
    for point in walls.tolist():
        memory[tuple(point)] = point
        if len(memory) > MAX_MEMORY_WALLS:
            raise ValueError('obstacle memory is full; no occupied cells were discarded')
    remembered = list(memory.values())
    cell = max(current_cell, old_cell)
    planned_clearance = max(.30, clearance) + TRACKING_RESERVE_M
    all_walls = np.asarray(remembered, dtype=float).reshape(-1, 2)

    # The guard stops the aircraft at or inside the planned clearance, often too
    # close for a sideways bump to curve around the obstacle. Try the detour
    # from the hover first (unless inside the clearance), then from checked
    # straight back-off points progressively farther from the nearest cell.
    need = planned_clearance+cell/math.sqrt(2)
    inside = bool(len(all_walls)) and float(
        np.linalg.norm(all_walls-np.asarray(current[:2]), axis=1).min()) < need-1e-9
    options = ([] if inside else [None]) + (list(BACK_OFF_MARGINS_M) if len(all_walls) else [])
    reach = offset + planned_clearance + cell/math.sqrt(2)
    hover_baseline, result, why = baseline, None, 'no valid detour'
    for margin in options:
        try:
            lead = [] if margin is None else [current]
            baseline = (hover_baseline if margin is None else
                        [_back_off(current, all_walls, cell, need, margin)] + suffix)
            check_bounds(baseline)
        except ValueError as exc:
            why = str(exc)
            continue
        # A deformation is bounded to offset from the baseline. A cell farther
        # than this expanded box cannot intersect any candidate. Keep it in the
        # returned flight guard memory even though geometry work can omit it.
        base_xy = np.asarray(lead + baseline, dtype=float)[:, :2]
        box_low, box_high = base_xy.min(axis=0), base_xy.max(axis=0)
        gap = np.maximum(np.maximum(box_low-all_walls, all_walls-box_high), 0.)
        relevant = all_walls[np.linalg.norm(gap, axis=1) <= reach+1e-9]
        if len(relevant) > MAX_WALLS:
            raise ValueError('too many nearby occupied cells for bounded replanning')
        result = deform_preview(baseline, relevant, clearance_m=planned_clearance,
                                max_offset_m=offset, cell_m=cell,
                                max_work_s=remaining_work(), pass_clearance_m=max(.30, clearance))
        if result.get('status') in ('adjusted', 'unchanged'):
            break
        why = str(result.get('reason', 'no valid detour'))
        result = None
    if result is None:
        raise ValueError('Remaining route blocked: ' + why)
    adjusted = result.get('points')
    if not isinstance(adjusted, list) or not 2 <= len(adjusted) <= MAX_SAMPLES:
        raise ValueError('replan returned no bounded route')
    adjusted = [_point(point, 3, 'adjusted point') for point in adjusted]
    if math.dist(adjusted[0], baseline[0]) > 1e-9 or math.dist(adjusted[-1], baseline[-1]) > 1e-9:
        raise ValueError('replan moved a preserved route endpoint')
    adjusted = lead + adjusted
    if len(adjusted) > MAX_SAMPLES:
        raise ValueError('replan returned no bounded route')
    check_bounds(adjusted)
    remaining_work()
    yaws = tangent_yaws(adjusted, initial_yaw=anchor['yaw_rad'])
    indices = (list(range(len(adjusted))) if len(adjusted) <= MAX_DISPLAY_POINTS else
               [i*(len(adjusted)-1)//(MAX_DISPLAY_POINTS-1) for i in range(MAX_DISPLAY_POINTS)])
    digest = hashlib.sha256(json.dumps(adjusted, separators=(',', ':'), allow_nan=False).encode()).hexdigest()[:24]
    remaining_work()
    observed_age = age + max(0., time.perf_counter()-started)
    if observed_age > MAX_DEPTH_AGE_S:
        raise ValueError('obstacle observations expired during replanning')
    return dict(points=adjusted, times_s=None, anchor=copy.deepcopy(anchor),
                reference=copy.deepcopy(reference), source=result['status'],
                next_index=next_index, observation_age_s=observed_age,
                avoidance=dict(clearance_m=max(.30, clearance),
                               planned_clearance_m=planned_clearance,
                               walls=remembered, cell_m=cell),
                display=dict(points=[list(adjusted[i]) for i in indices],
                             yaws=[yaws[i] for i in indices], version='replan:'+digest))

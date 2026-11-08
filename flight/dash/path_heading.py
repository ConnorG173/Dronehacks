"""Geometric path headings; no state, ROS dependency, or flight commands.

Headings use the input XY frame, counterclockwise from +X. We use outgoing
distance-window secants, not central differences: a reversal must not average
to a zero vector, and a sharp corner must not acquire a diagonal heading.
Direction windows never cross stationary/vertical segments or a >45-degree
change between neighboring segment directions. This preserves deliberate sharp
turns; it does not promise a dynamically feasible yaw transition at a cusp.
"""
import math

import numpy as np


MIN_DIRECTION_M = .01
_STATIONARY_M = 1e-9
_CORNER_COS = math.cos(math.pi/4)


def wrap_angle(angle):
    """Finite scalar angle -> principal angle in [-pi, pi]."""
    angle = float(angle)
    if not math.isfinite(angle):
        raise ValueError('heading must be finite')
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_quaternion(yaw):
    """Yaw-only ROS quaternion as (x, y, z, w), in the same frame as yaw."""
    yaw = wrap_angle(yaw)
    return (0., 0., math.sin(yaw/2), math.cos(yaw/2))


def tangent_yaws(points_xyz, initial_yaw=0.0):
    """Return one finite, unwrapped geometric heading per XYZ point.

Movement shorter than a 1 cm net XY direction window is ignored. Gentle
curves may use up to 2 cm of forward arc to obtain that net displacement.
Internal stationary/vertical stretches hold the last defined heading. A
stopped start uses the first meaningful outgoing direction; a stopped finish
holds the last heading. An entirely stationary/tiny path retains initial_yaw.
Inputs are never modified. The added unwrapping removes +/-pi discontinuities,
but actual sharp geometric turns remain sharp.
"""
    initial = float(initial_yaw)
    if not math.isfinite(initial):
        raise ValueError('initial heading must be finite')
    points = np.asarray(points_xyz, dtype=float)
    if points.shape == (0,) or points.shape == (0,3):
        return []
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('path must contain finite XYZ points')
    n = len(points)
    raw = np.full(n, np.nan)
    delta = np.diff(points[:,:2], axis=0)
    lengths = np.hypot(delta[:,0], delta[:,1])
    if not np.isfinite(lengths).all():
        raise ValueError('path displacement must be finite')
    directions = delta/np.maximum(lengths[:,None], _STATIONARY_M)

    start = 0
    while start < n-1:
        if lengths[start] <= _STATIONARY_M:
            start += 1
            continue
        stop = start+1
        while (stop < n-1 and lengths[stop] > _STATIONARY_M
               and float(np.dot(directions[stop-1],directions[stop])) >= _CORNER_COS):
            stop += 1
        arc = np.r_[0., np.cumsum(lengths[start:stop])]
        for offset in range(stop-start):
            if arc[-1]-arc[offset] < MIN_DIRECTION_M-1e-12:
                break
            for distance in (MIN_DIRECTION_M, 1.5*MIN_DIRECTION_M, 2*MIN_DIRECTION_M):
                target = min(arc[offset]+distance, arc[-1])
                end = min(int(np.searchsorted(arc,target-1e-12,side='left')),len(arc)-1)
                vector = points[start+end,:2]-points[start+offset,:2]
                if np.hypot(*vector) >= MIN_DIRECTION_M-1e-12:
                    raw[start+offset] = math.atan2(float(vector[1]),float(vector[0]))
                    break
        start = stop

    defined = np.flatnonzero(np.isfinite(raw))
    heading = initial
    if len(defined):
        heading += wrap_angle(float(raw[defined[0]])-heading)
    result = []
    for value in raw:
        if math.isfinite(value):
            heading += wrap_angle(float(value)-heading)
        result.append(float(heading))
    return result

"""bench.analysis - numbers from recorded test samples. PURE: no ROS."""
from __future__ import annotations

import math
import statistics


def tracking(samples):
    """samples: [(t, sp_p(3), pos(3))] -> per-axis and 3D RMS / max error (m) and mean lag (s)."""
    if not samples:
        return None
    errs = [[s[2][i] - s[1][i] for i in range(3)] for s in samples]
    res = {}
    for i, ax in enumerate("xyz"):
        col = [e[i] for e in errs]
        res[f"rms_{ax}"] = math.sqrt(sum(c * c for c in col) / len(col))
        res[f"max_{ax}"] = max(abs(c) for c in col)
    e3 = [math.sqrt(sum(c * c for c in e)) for e in errs]
    res["rms_3d"] = math.sqrt(sum(c * c for c in e3) / len(e3))
    res["max_3d"] = max(e3)
    res["lag_s"] = estimate_lag(samples)
    return res


def estimate_lag(samples, max_lag_s=1.0):
    """Delay (s) that best aligns the horizontal position with the setpoint (cross-correlation by
    brute force over a grid of shifts).  None if the setpoint barely moved."""
    if len(samples) < 20:
        return None
    t = [s[0] for s in samples]
    dt = (t[-1] - t[0]) / (len(t) - 1)
    if dt <= 0:
        return None
    sx = [s[1][0] for s in samples]
    sy = [s[1][1] for s in samples]
    if max(sx) - min(sx) < 0.05 and max(sy) - min(sy) < 0.05:
        return None
    px = [s[2][0] for s in samples]
    py = [s[2][1] for s in samples]
    best, best_k = None, 0
    for k in range(0, int(max_lag_s / dt) + 1):
        n = len(samples) - k
        if n < 10:
            break
        cost = sum((px[i + k] - sx[i]) ** 2 + (py[i + k] - sy[i]) ** 2 for i in range(n)) / n
        if best is None or cost < best:
            best, best_k = cost, k
    return best_k * dt


def step_response(times, values, t_step, start, target, settle_band=0.1):
    """Metrics for one step of `values` (same units as start/target) beginning at t_step.

    rise_s       10 % -> 90 % of the change
    overshoot    fraction of the change beyond the target
    settle_s     time after t_step until it stays within settle_band x |change| of target
    ss_err       mean |value - target| over the last 25 % of the window
    """
    change = target - start
    if abs(change) < 1e-9:
        return None
    w = [(t - t_step, (v - start) / change) for t, v in zip(times, values) if t >= t_step]
    if not w:
        return None
    t10 = next((t for t, u in w if u >= 0.1), None)
    t90 = next((t for t, u in w if u >= 0.9), None)
    peak = max(u for _, u in w)
    settle = None
    for i in range(len(w)):
        if all(abs(u - 1.0) <= settle_band for _, u in w[i:]):
            settle = w[i][0]
            break
    tail = w[int(len(w) * 0.75):] or w[-1:]
    return {
        "rise_s": None if t10 is None or t90 is None else t90 - t10,
        "overshoot": max(0.0, peak - 1.0),
        "settle_s": settle,
        "ss_err": statistics.fmean(abs(u - 1.0) for _, u in tail) * abs(change),
    }


def drift(samples):
    """samples: [(t, (x, y, z))] of a vehicle that is NOT moving -> noise and drift."""
    if len(samples) < 2:
        return None
    t0, p0 = samples[0]
    t1, p1 = samples[-1]
    cols = list(zip(*[p for _, p in samples]))
    span = max(max(c) - min(c) for c in cols)
    std = max(statistics.pstdev(c) for c in cols)
    net = math.dist(p0, p1)
    return {"duration_s": t1 - t0, "span_m": span, "noise_std_m": std, "net_drift_m": net,
            "drift_rate_mps": net / (t1 - t0) if t1 > t0 else None}


def body_displacement(p0, p1, yaw0):
    """NED displacement p1 - p0 expressed as (forward, right, down) of a vehicle with heading yaw0."""
    dx, dy, dz = (p1[i] - p0[i] for i in range(3))
    c, s = math.cos(yaw0), math.sin(yaw0)
    return (c * dx + s * dy, -s * dx + c * dy, dz)


def frame_check(direction, disp, min_m=0.3, max_cross_frac=0.5):
    """Did a hand-carried move in `direction` (forward/back/right/left/up/down) show up correctly?

    disp = (forward, right, down) displacement.  Pass if the commanded axis moved at least
    min_m in the right sign and the other two moved less than max_cross_frac of that."""
    axis, sign = {"forward": (0, 1), "back": (0, -1), "right": (1, 1), "left": (1, -1),
                  "down": (2, 1), "up": (2, -1)}[direction]
    along = disp[axis] * sign
    cross = max(abs(disp[i]) for i in range(3) if i != axis)
    if along < min_m:
        if along <= -min_m:
            return False, f"moved {along:+.2f} m: OPPOSITE sign - axis flipped in the frame conversion"
        big = max(range(3), key=lambda i: abs(disp[i]))
        if abs(disp[big]) >= min_m:
            names = ("forward", "right", "down")
            return False, (f"moved {disp[big]:+.2f} m along {names[big]} instead: axes swapped "
                           f"in the frame conversion (or the move was not {direction})")
        return False, f"moved only {along:+.2f} m (need {min_m:.2f}): estimate not following, or move too small"
    if cross > max_cross_frac * along:
        return False, f"moved {along:+.2f} m but {cross:.2f} m sideways: frame rotated / heading wrong"
    return True, f"moved {along:+.2f} m, {cross:.2f} m cross-axis"

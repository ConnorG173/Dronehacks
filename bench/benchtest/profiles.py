"""bench.profiles - setpoint profiles for tethered tests. PURE: no ROS, no clock reads.

Every profile is expressed RELATIVE to the hover point where the test starts, in the
vehicle's start frame:  f = forward, r = right, d = down (metres), yaw relative to the
start heading (rad, clockwise positive, like PX4).  `to_ned` turns a relative setpoint
into PX4 local NED.  Each profile gives position, velocity AND acceleration, so the same
feedforward path the real trajectory follower will use is exercised on the tether.

A profile is any object with:
    duration  (s)
    sample(t) -> Setpoint   (t clamped to [0, duration]; at t >= duration it holds the end)
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass


@dataclass
class Setpoint:
    p: tuple = (0.0, 0.0, 0.0)
    v: tuple = (0.0, 0.0, 0.0)
    a: tuple = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    yaw_rate: float = 0.0


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def to_ned(sp: Setpoint, origin, yaw0) -> Setpoint:
    """Relative (forward, right, down) setpoint -> PX4 local NED, around origin (x, y, z) and yaw0."""
    c, s = math.cos(yaw0), math.sin(yaw0)

    def rot(vec, offset=(0.0, 0.0, 0.0)):
        f, r, d = vec
        return (offset[0] + c * f - s * r, offset[1] + s * f + c * r, offset[2] + d)

    return Setpoint(rot(sp.p, origin), rot(sp.v), rot(sp.a), wrap(yaw0 + sp.yaw), sp.yaw_rate)


# --------------------------------------------------------------------------- building blocks
def minjerk(u):
    """Quintic smoothstep s(u) and its first two derivatives w.r.t. u, u in [0, 1]."""
    u = min(max(u, 0.0), 1.0)
    s = u ** 3 * (10 - 15 * u + 6 * u * u)
    ds = 30 * u * u * (1 - u) ** 2
    dds = 60 * u * (1 - u) * (1 - 2 * u)
    return s, ds, dds


class Hover:
    """Hold the hover point for `duration` seconds."""

    def __init__(self, duration):
        self.duration = float(duration)

    def sample(self, t):
        return Setpoint()


AXES = {"f": 0, "r": 1, "d": 2}


class Steps:
    """A sequence of moves away from and back to the hover point.

    moves: list of (axis, amount) with axis in f / r / d / yaw (metres or degrees).
    Each move goes out, holds `hold` s, comes back, holds `hold` s.
    shape 'smooth': minimum-jerk transitions over `move_time` with matching velocity and
    acceleration feedforward.  shape 'step': instantaneous setpoint jumps (classic step
    response; PX4 limits the speed with MPC_XY_VEL_MAX / MPC_Z_VEL_MAX_*).
    """

    def __init__(self, moves, hold=4.0, move_time=1.0, shape="smooth"):
        if shape not in ("smooth", "step"):
            raise ValueError("shape must be 'smooth' or 'step'")
        self.shape, self.hold = shape, float(hold)
        self.move_time = float(move_time) if shape == "smooth" else 0.0
        self.segments = []  # (t_start, axis, from, to)
        t = 0.0
        for axis, amount in moves:
            if axis not in AXES and axis != "yaw":
                raise ValueError(f"unknown axis {axis!r}")
            amount = math.radians(amount) if axis == "yaw" else float(amount)
            for a, b in ((0.0, amount), (amount, 0.0)):
                self.segments.append((t, axis, a, b))
                t += self.move_time + self.hold
        self.duration = t

    def sample(self, t):
        t = min(max(t, 0.0), self.duration)
        p, v, a = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        yaw = yaw_rate = 0.0
        seg = None
        for s in self.segments:
            if s[0] <= t:
                seg = s
        if seg is None:
            return Setpoint()
        t0, axis, x0, x1 = seg
        if self.shape == "step" or t - t0 >= self.move_time:
            x, dx, ddx = x1, 0.0, 0.0
        else:
            s, ds, dds = minjerk((t - t0) / self.move_time)
            x = x0 + (x1 - x0) * s
            dx = (x1 - x0) * ds / self.move_time
            ddx = (x1 - x0) * dds / self.move_time ** 2
        if axis == "yaw":
            yaw, yaw_rate = x, dx
        else:
            i = AXES[axis]
            p[i], v[i], a[i] = x, dx, ddx
        return Setpoint(tuple(p), tuple(v), tuple(a), yaw, yaw_rate)


class Circle:
    """Horizontal circle starting and ending at the hover point, with smooth speed ramps.

    The angular speed ramps 0 -> w0 over `ramp` s (quintic), holds, then ramps back to 0,
    so position, velocity and acceleration are continuous and start/end at rest.  The
    circle's centre is `radius` to the right of the hover point (r > 0) or left (r < 0
    with direction -1).  Yaw is held at the start heading (camera keeps looking forward).
    """

    def __init__(self, radius=0.3, period=6.0, laps=2.0, ramp=3.0, direction=1):
        self.R, self.w0 = float(radius), 2.0 * math.pi / float(period)
        self.ramp, self.dir = float(ramp), 1 if direction >= 0 else -1
        phase_ramps = 2 * self.w0 * self.ramp / 2.0  # each ramp covers w0 * ramp / 2 of phase
        cruise_phase = max(0.0, 2.0 * math.pi * float(laps) - phase_ramps)
        self.cruise = cruise_phase / self.w0
        self.duration = 2 * self.ramp + self.cruise

    def _phase(self, t):
        """phase, angular rate, angular acceleration at time t."""
        T, w0 = self.ramp, self.w0
        t = min(max(t, 0.0), self.duration)
        if t < T:
            u = t / T
            s, ds, _ = minjerk(u)
            integ = (2.5 * u ** 4 - 3 * u ** 5 + u ** 6) * T      # integral of s over [0, t]
            return w0 * integ, w0 * s, w0 * ds / T
        ph1 = w0 * T / 2.0
        if t < T + self.cruise:
            return ph1 + w0 * (t - T), w0, 0.0
        u = (t - T - self.cruise) / T
        s, ds, _ = minjerk(u)
        integ = (u - (2.5 * u ** 4 - 3 * u ** 5 + u ** 6)) * T   # integral of (1 - s)
        return ph1 + w0 * self.cruise + w0 * integ, w0 * (1 - s), -w0 * ds / T

    def sample(self, t):
        ph, w, al = self._phase(t)
        R, k = self.R, self.dir
        # p = R (sin ph, k (1 - cos ph)): starts at 0 moving forward, centre at (0, k R)
        p = (R * math.sin(ph), k * R * (1 - math.cos(ph)), 0.0)
        v = (R * w * math.cos(ph), k * R * w * math.sin(ph), 0.0)
        a = (R * (al * math.cos(ph) - w * w * math.sin(ph)),
             k * R * (al * math.sin(ph) + w * w * math.cos(ph)), 0.0)
        return Setpoint(p, v, a, 0.0, 0.0)


class PathFile:
    """A gcopter_planner path (path.bin or path.csv), replayed relative to the hover point.

    Planner rows are PX4 local NED with +x = the start heading (planner/README.md), so
    subtracting the first row gives exactly this module's forward/right/down frame.
    speed < 1 slows the path down: p(t) = P(speed t), v = speed V, a = speed^2 A.
    """

    def __init__(self, path, speed=1.0):
        if not 0.0 < speed <= 1.0:
            raise ValueError("speed must be in (0, 1]")
        self.rows = load_path(path)
        if len(self.rows) < 2:
            raise ValueError(f"{path}: fewer than 2 rows")
        self.speed = float(speed)
        r0 = self.rows[0]
        self.p0, self.yaw0 = r0[1:4], r0[10]
        self.T = self.rows[-1][0]
        self.duration = self.T / self.speed
        self._i = 0

    def sample(self, t):
        tau = min(max(t, 0.0), self.duration) * self.speed
        rows = self.rows
        if tau < rows[self._i][0]:
            self._i = 0
        while self._i < len(rows) - 2 and rows[self._i + 1][0] <= tau:
            self._i += 1
        r0, r1 = rows[self._i], rows[self._i + 1]
        h = r1[0] - r0[0]
        u = 0.0 if h <= 0 else min(max((tau - r0[0]) / h, 0.0), 1.0)
        row = [x0 + (x1 - x0) * u for x0, x1 in zip(r0, r1)]
        row[10] = r0[10] + wrap(r1[10] - r0[10]) * u
        k = self.speed
        p = tuple(row[1 + i] - self.p0[i] for i in range(3))
        v = tuple(k * row[4 + i] for i in range(3))
        a = tuple(k * k * row[7 + i] for i in range(3))
        return Setpoint(p, v, a, wrap(row[10] - self.yaw0), k * row[11])


def load_path(path):
    """Rows of (t, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate) from path.bin or path.csv."""
    if str(path).endswith(".csv"):
        rows = []
        with open(path) as f:
            for line in f:
                parts = line.strip().split(",")
                try:
                    vals = [float(x) for x in parts]
                except ValueError:
                    continue                          # header
                if len(vals) == 12:
                    rows.append(vals)
        return rows
    with open(path, "rb") as f:
        raw = f.read()
    if raw[:8] != b"DHPATH\0\0":
        raise ValueError(f"{path}: not a DHPATH file")
    version, ncols, nrows = struct.unpack_from("<IIQ", raw, 8)
    if version != 1 or ncols != 12:
        raise ValueError(f"{path}: unsupported version {version} / {ncols} columns")
    vals = struct.unpack_from(f"<{nrows * 12}d", raw, 32)
    return [list(vals[i * 12:(i + 1) * 12]) for i in range(nrows)]


def extent(profile, dt=0.02):
    """Max horizontal distance from the hover point, max height above (positive) and below the hover point,
    max speed and max horizontal acceleration over the whole profile."""
    xy = up = down = vmax = amax = 0.0
    n = int(profile.duration / dt) + 1
    for i in range(n + 1):
        sp = profile.sample(min(i * dt, profile.duration))
        xy = max(xy, math.hypot(sp.p[0], sp.p[1]))
        up, down = max(up, -sp.p[2]), max(down, sp.p[2])
        vmax = max(vmax, math.sqrt(sum(c * c for c in sp.v)))
        amax = max(amax, math.hypot(sp.a[0], sp.a[1]))
    return {"xy": xy, "up": up, "down": down, "vmax": vmax, "amax": amax}

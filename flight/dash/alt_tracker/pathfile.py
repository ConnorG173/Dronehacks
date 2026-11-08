"""Load the planner's DHPATH file and serve it as a time-indexed trajectory in map ENU.

Format: planner/README.md "Output". Rows: t, x, y, z, vx, vy, vz, ax, ay, az, yaw,
yaw_rate in planner NED. Converted to the map frame once at load.
"""
import struct

import numpy as np

from . import frames

COLUMNS = 12


def read_dhpath(path):
    with open(path, 'rb') as handle:
        raw = handle.read()
    if raw[:8] != b'DHPATH\0\0':
        raise ValueError(f'{path}: not a DHPATH file')
    version, ncols, nrows, dt = struct.unpack_from('<IIQd', raw, 8)
    if version != 1 or ncols != COLUMNS:
        raise ValueError(f'{path}: unsupported DHPATH version {version} / {ncols} columns')
    rows = np.frombuffer(raw, '<f8', count=nrows * ncols, offset=32).reshape(nrows, ncols)
    if not np.all(np.isfinite(rows)):
        raise ValueError(f'{path}: non-finite values')
    return rows.copy(), dt


def write_dhpath(path, rows, dt):
    rows = np.asarray(rows, dtype='<f8')
    with open(path, 'wb') as handle:
        handle.write(b'DHPATH\0\0' + struct.pack('<IIQd', 1, COLUMNS, rows.shape[0], dt))
        handle.write(rows.tobytes())


class Trajectory:
    """Time-sampled trajectory in map ENU with linear interpolation."""

    def __init__(self, t, p, v, a, yaw, yaw_rate):
        self.t = np.asarray(t, float)
        self.p = np.asarray(p, float)
        self.v = np.asarray(v, float)
        self.a = np.asarray(a, float)
        self.yaw = np.unwrap(np.asarray(yaw, float))
        self.yaw_rate = np.asarray(yaw_rate, float)
        if self.t.ndim != 1 or len(self.t) < 2 or np.any(np.diff(self.t) <= 0):
            raise ValueError('trajectory time must be strictly increasing with >= 2 rows')
        seg = np.linalg.norm(np.diff(self.p, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])  # arc length per row

    @classmethod
    def from_planner(cls, rows, start_pos, psi0):
        rows = np.asarray(rows)
        return cls(
            rows[:, 0],
            frames.planner_ned_to_map_points(rows[:, 1:4], start_pos, psi0),
            frames.planner_ned_to_map_vectors(rows[:, 4:7], psi0),
            frames.planner_ned_to_map_vectors(rows[:, 7:10], psi0),
            frames.planner_yaw_to_map(rows[:, 10], psi0),
            -rows[:, 11],
        )

    @classmethod
    def load(cls, path, course):
        rows, _ = read_dhpath(path)
        return cls.from_planner(rows, course.start_pos, course.start_heading_rad)

    @property
    def duration(self):
        return float(self.t[-1])

    @property
    def length(self):
        return float(self.s[-1])

    def _idx(self, tau):
        tau = min(max(float(tau), self.t[0]), self.t[-1])
        i = int(np.searchsorted(self.t, tau, side='right')) - 1
        i = min(max(i, 0), len(self.t) - 2)
        w = (tau - self.t[i]) / (self.t[i + 1] - self.t[i])
        return i, w

    def sample(self, tau):
        """(p, v, a, yaw, yaw_rate) at path time tau (clamped to [0, T])."""
        i, w = self._idx(tau)
        lerp = lambda arr: arr[i] * (1.0 - w) + arr[i + 1] * w  # noqa: E731
        return lerp(self.p), lerp(self.v), lerp(self.a), float(lerp(self.yaw)), float(lerp(self.yaw_rate))

    def position(self, tau):
        i, w = self._idx(tau)
        return self.p[i] * (1.0 - w) + self.p[i + 1] * w

    def tangent(self, tau, eps=1e-3):
        """Unit direction of travel at tau; falls back to the nearest moving sample."""
        _, v, _, _, _ = self.sample(tau)
        n = np.linalg.norm(v)
        if n > eps:
            return v / n
        i, _ = self._idx(tau)
        speeds = np.linalg.norm(self.v, axis=1)
        moving = np.nonzero(speeds > eps)[0]
        if len(moving) == 0:
            return np.zeros(3)
        j = moving[np.argmin(np.abs(moving - i))]
        return self.v[j] / speeds[j]

    def nearest(self, p, tau_hint=None, window=2.0):
        """Path time of the closest sample to p (within +-window s of tau_hint if given)."""
        p = np.asarray(p)
        if tau_hint is None:
            lo, hi = 0, len(self.t)
        else:
            lo = int(np.searchsorted(self.t, tau_hint - window))
            hi = int(np.searchsorted(self.t, tau_hint + window)) + 1
        d = np.linalg.norm(self.p[lo:hi] - p, axis=1)
        k = lo + int(np.argmin(d))
        return float(self.t[k]), float(d[k - lo])

    @classmethod
    def hold(cls, p, yaw, seconds, dt=0.01):
        """Stand still at p (map frame) with heading yaw for `seconds`."""
        n = max(2, int(round(seconds / dt)) + 1)
        t = np.linspace(0.0, max(seconds, dt), n)
        z = np.zeros((n, 3))
        return cls(t, np.tile(np.asarray(p, float), (n, 1)), z, z.copy(), np.full(n, float(yaw)), np.zeros(n))

    def with_hold(self, seconds, dt=0.01):
        """Copy with `seconds` of standing still at the final point appended (velocity and
        acceleration zero there; the plan already ends at rest)."""
        if seconds <= 0.0:
            return self
        h = Trajectory.hold(self.p[-1], self.yaw[-1], seconds, dt)
        cat = lambda a, b: np.concatenate([a, b[1:]])  # noqa: E731
        return Trajectory(cat(self.t, h.t + self.t[-1]), cat(self.p, h.p), cat(self.v, h.v), cat(self.a, h.a),
                          cat(self.yaw, h.yaw), cat(self.yaw_rate, h.yaw_rate))

    def clearance_retimed(self, clearance, slow_below, full_slow_at, k_max, smooth_s=1.0):
        """Copy slowed where the plan runs close to obstacles: the local time stretch k goes from 1 at
        clearance >= slow_below to k_max at clearance <= full_slow_at, smoothed over smooth_s (centred
        moving average) so the re-timed velocity and acceleration stay continuous. Corner cutting grows
        with speed, so this buys clearance exactly in the narrow parts (e.g. a chicane channel).
        clearance: callable (N, 3) -> (N,) distance to the nearest obstacle (m)."""
        if k_max <= 1.0 or slow_below <= full_slow_at:
            return self
        c = np.asarray(clearance(self.p), float)
        k = 1.0 + (k_max - 1.0) * np.clip((slow_below - c) / (slow_below - full_slow_at), 0.0, 1.0)
        dt = np.diff(self.t)
        n = max(1, int(round(smooth_s / max(float(np.median(dt)), 1e-6))))
        if n > 1:
            pad = np.pad(k, (n // 2, n - 1 - n // 2), mode='edge')
            k = np.convolve(pad, np.ones(n) / n, mode='valid')
        km = 0.5 * (k[1:] + k[:-1])
        t = np.concatenate([[0.0], np.cumsum(dt * km)])
        # d(v/k)/d(new_time) includes the changing stretch, not just a/k**2.
        dk_dt = np.gradient(k, self.t)
        acceleration = self.a / k[:, None] ** 2 - self.v * (dk_dt / k ** 3)[:, None]
        return Trajectory(t, self.p, self.v / k[:, None], acceleration, self.yaw, self.yaw_rate / k)

    def time_scaled(self, k):
        """Uniformly slowed (k > 1) or sped up (k < 1) copy: exact for any path."""
        return Trajectory(self.t * k, self.p, self.v / k, self.a / (k * k), self.yaw, self.yaw_rate / k)

    def decimated(self, step):
        idx = np.arange(0, len(self.t), max(1, int(step)))
        if idx[-1] != len(self.t) - 1:
            idx = np.append(idx, len(self.t) - 1)
        return idx

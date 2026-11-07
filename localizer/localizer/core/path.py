"""path.bin loading and the planned path in the course frame. Pure numpy.

path.bin (planner/README.md): DHPATH v1, 12 float64 columns
t, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate in PX4-local NED at the pad
(origin StartPos on the ground, +x along StartHeadingDeg, z down).

Course frame: the planner map frame. x, y metres, z up = height above the floor, yaw CCW from +x.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def load_path(path):
    """path.bin -> (R, 12) rows t, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate (NED at pad)."""
    raw = open(path, "rb").read()
    if raw[:8] != b"DHPATH\0\0":
        raise ValueError(f"{path}: not a DHPATH file")
    version, cols = np.frombuffer(raw, "<u4", count=2, offset=8)
    if version != 1 or cols != 12:
        raise ValueError(f"{path}: version {version}, {cols} columns (expected 1, 12)")
    n = int(np.frombuffer(raw, "<u8", count=1, offset=16)[0])
    rows = np.frombuffer(raw, "<f8", count=n * 12, offset=32).reshape(n, 12)
    if not np.isfinite(rows).all():
        raise ValueError(f"{path}: non-finite values")
    return rows.copy()


@dataclass
class CoursePath:
    t: np.ndarray          # (R,) s
    xy: np.ndarray         # (R, 2) course
    z: np.ndarray          # (R,) height above the floor
    yaw: np.ndarray        # (R,) course heading, unwrapped
    speed: np.ndarray      # (R,) horizontal+vertical speed, m/s
    vel: np.ndarray        # (R, 3) course x, y, z-up velocity, m/s (path time)
    acc: np.ndarray        # (R, 3) course acceleration, m/s^2 (path time)
    yaw_rate: np.ndarray   # (R,) course yaw rate (CCW), rad/s (path time)

    @property
    def duration(self):
        return float(self.t[-1])

    def at(self, tau):
        """Interpolated (xy, z, yaw) at path time tau (clamped to [0, duration])."""
        tau = min(max(float(tau), 0.0), self.duration)
        xy = np.array([np.interp(tau, self.t, self.xy[:, 0]), np.interp(tau, self.t, self.xy[:, 1])])
        return xy, float(np.interp(tau, self.t, self.z)), float(np.interp(tau, self.t, self.yaw))

    def state(self, tau):
        """(p (3,), v (3,), a (3,), yaw) at path time tau, course frame, derivatives w.r.t. tau.
        v and a are zero outside [0, duration] (the path starts and ends at rest)."""
        xy, z, yaw = self.at(tau)
        if not 0.0 <= tau <= self.duration:
            return np.r_[xy, z], np.zeros(3), np.zeros(3), yaw
        v = np.array([np.interp(tau, self.t, self.vel[:, k]) for k in range(3)])
        a = np.array([np.interp(tau, self.t, self.acc[:, k]) for k in range(3)])
        return np.r_[xy, z], v, a, yaw

    def yaw_rate_at(self, tau):
        if not 0.0 <= tau <= self.duration:
            return 0.0
        return float(np.interp(tau, self.t, self.yaw_rate))


def path_to_course(rows, start_xy, start_heading_rad, start_z=0.0):
    """NED-at-pad rows -> CoursePath (inverse of planner/README.md 'Frames')."""
    c, s = math.cos(start_heading_rad), math.sin(start_heading_rad)

    def to_course(n):           # NED vector columns (x, y, z) -> course (dx, dy, up), no offset
        # x_ned = c dx + s dy, y_ned = s dx - c dy  ->  the same matrix inverts it
        return np.stack([c * n[:, 0] + s * n[:, 1], s * n[:, 0] - c * n[:, 1], -n[:, 2]], 1)

    d = to_course(rows[:, 1:4])
    yaw = np.unwrap(start_heading_rad - rows[:, 10])        # map heading theta = psi0 - yaw_ned
    return CoursePath(t=rows[:, 0].copy(), xy=d[:, :2] + np.asarray(start_xy, float)[:2],
                      z=start_z + d[:, 2], yaw=yaw, speed=np.linalg.norm(rows[:, 4:7], axis=1),
                      vel=to_course(rows[:, 4:7]), acc=to_course(rows[:, 7:10]),
                      yaw_rate=-rows[:, 11].copy())


def time_scale_for(path: CoursePath, max_speed_mps):
    """Play the path slower so its peak speed is at most max_speed_mps (never faster)."""
    vmax = float(path.speed.max())
    return 1.0 if vmax <= max_speed_mps else max_speed_mps / vmax

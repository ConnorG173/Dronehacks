"""path_streamer's logic, PURE numpy (the ROS node is a thin shell around it).

Frames:
  path.bin  PX4-local-NED at the pad (planner/README.md): origin StartPos, +x along StartHeadingDeg
  course    planner map frame: x, y metres, z up = height above the floor, yaw CCW from +x
  odom      cuVSLAM odom (what /skybeta/setpoint/pose is in): x, y, yaw CCW
  T         course <- odom, (x, y, yaw): p_course = R(T.yaw) p_odom + (T.x, T.y)

Setpoint z is height above the floor straight from the path; it never goes through T.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


# ----------------------------------------------------------------------------- path

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

    @property
    def duration(self):
        return float(self.t[-1])

    def at(self, tau):
        """Interpolated (xy, z, yaw) at path time tau (clamped to [0, duration])."""
        tau = min(max(float(tau), 0.0), self.duration)
        xy = np.array([np.interp(tau, self.t, self.xy[:, 0]), np.interp(tau, self.t, self.xy[:, 1])])
        return xy, float(np.interp(tau, self.t, self.z)), float(np.interp(tau, self.t, self.yaw))


def path_to_course(rows, start_xy, start_heading_rad, start_z=0.0):
    """NED-at-pad rows -> CoursePath (inverse of planner/README.md 'Frames')."""
    c, s = math.cos(start_heading_rad), math.sin(start_heading_rad)
    xn, yn, zn = rows[:, 1], rows[:, 2], rows[:, 3]
    # x_ned = c dx + s dy, y_ned = s dx - c dy  ->  the same matrix inverts it
    dx = c * xn + s * yn
    dy = s * xn - c * yn
    yaw = np.unwrap(start_heading_rad - rows[:, 10])        # map heading theta = psi0 - yaw_ned
    return CoursePath(t=rows[:, 0].copy(), xy=np.stack([dx + start_xy[0], dy + start_xy[1]], 1),
                      z=start_z - zn, yaw=yaw, speed=np.linalg.norm(rows[:, 4:7], axis=1))


# ----------------------------------------------------------------------------- T = course <- odom

@dataclass
class Transform2D:
    x: float
    y: float
    yaw: float

    def to_course(self, xy_odom, yaw_odom=None):
        p = _rot(self.yaw) @ np.asarray(xy_odom, float)[:2] + (self.x, self.y)
        return p if yaw_odom is None else (p, wrap(yaw_odom + self.yaw))

    def to_odom(self, xy_course, yaw_course=None):
        p = _rot(-self.yaw) @ (np.asarray(xy_course, float)[:2] - (self.x, self.y))
        return p if yaw_course is None else (p, wrap(yaw_course - self.yaw))


def pad_init(odom_xy, odom_yaw, odom_tilt_deg, px4_tilt_deg, pad_xy, pad_yaw,
             max_odom_offset_m=0.15, max_odom_yaw_deg=5.0, max_tilt_diff_deg=3.0):
    """T0 = pad pose o odom pose^-1, or (None, reason). Assumes the drone sits on the launch mark
    facing StartHeadingDeg (HALF-ASSED: nothing measures that; the brief's wide registration
    cross-check against the CAD is not done)."""
    off = float(np.hypot(*np.asarray(odom_xy, float)[:2]))
    if off > max_odom_offset_m:
        return None, f"odom not near zero ({off:.2f} m): restart the stack on the pad"
    if abs(math.degrees(wrap(odom_yaw))) > max_odom_yaw_deg:
        return None, f"odom yaw {math.degrees(wrap(odom_yaw)):.1f} deg not near zero"
    if abs(odom_tilt_deg - px4_tilt_deg) > max_tilt_diff_deg:
        return None, (f"odom tilt {odom_tilt_deg:.1f} deg vs PX4 {px4_tilt_deg:.1f} deg: "
                      f"cuVSLAM attitude disagrees with the IMU")
    yaw = wrap(pad_yaw - odom_yaw)
    t = np.asarray(pad_xy, float) - _rot(yaw) @ np.asarray(odom_xy, float)[:2]
    return Transform2D(float(t[0]), float(t[1]), yaw), "ok"


# ----------------------------------------------------------------------------- carrot

class Carrot:
    """Time-pausing carrot: path time advances at `scale` x real time while the vehicle is within
    `max_err_m` of the commanded point, and stops while it is further away."""

    def __init__(self, path: CoursePath, scale=1.0, max_err_m=0.6):
        self.path, self.scale, self.max_err = path, float(scale), float(max_err_m)
        self.tau = 0.0
        self.paused = False

    @property
    def done(self):
        return self.tau >= self.path.duration

    def step(self, dt, vehicle_xyz_course):
        xy, z, _ = self.path.at(self.tau)
        err = float(np.linalg.norm(np.r_[xy, z] - np.asarray(vehicle_xyz_course, float)))
        self.paused = err > self.max_err
        if not self.paused:
            self.tau = min(self.tau + float(dt) * self.scale, self.path.duration)
        return self.path.at(self.tau), err


def time_scale_for(path: CoursePath, max_speed_mps):
    """Play the path slower so its peak speed is at most max_speed_mps (never faster)."""
    vmax = float(path.speed.max())
    return 1.0 if vmax <= max_speed_mps else max_speed_mps / vmax


# ----------------------------------------------------------------------------- geofence

def inside_fence(xy_course, cage_half_m=3.049, margin_m=0.3):
    lim = cage_half_m - margin_m
    return bool(np.all(np.abs(np.asarray(xy_course, float)[:2]) <= lim))


def clamp_to_fence(xy_course, cage_half_m=3.049, margin_m=0.3):
    lim = cage_half_m - margin_m
    return np.clip(np.asarray(xy_course, float)[:2], -lim, lim)

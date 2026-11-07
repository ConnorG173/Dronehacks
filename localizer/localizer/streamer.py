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
    vel: np.ndarray = None  # (R, 3) course x, y, z-up velocity, m/s (path time)
    acc: np.ndarray = None  # (R, 3) course acceleration, m/s^2 (path time)

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
                      vel=to_course(rows[:, 4:7]), acc=to_course(rows[:, 7:10]))


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


# ----------------------------------------------------------------------------- path clock + lead
#
# PX4 v1.16 Offboard has no waypoint logic: mc_pos_control (PositionControl.cpp) tracks whatever
# TrajectorySetpoint is current, with
#   v_sp = Kp (p_sp - x) + v_ff                    Kp = MPC_XY_P 0.95, MPC_Z_P 1.0 (defaults)
#   a_sp = Kv (v_sp - v) + I - D dv/dt + a_ff      Kv = MPC_XY_VEL_P_ACC 1.8
# flight_interface sends position only (v_ff = a_ff = NaN), so a setpoint ON the vehicle makes
# it lag by v / Kp (1.05 s of travel). Sending the path point a time L = 1 / Kp ahead gives
#   Kp (p(tau + L) - x) = Kp (p(tau) - x) + v + a / (2 Kp) + ...
# i.e. velocity feedforward plus roughly the acceleration feedforward (a / 1.80 vs the ideal
# a / Kv = a / 1.8 through the velocity loop). Unlike the algebraic p + v/Kp + a/(Kp Kv), the
# lead stays a point of the planned, collision-checked path: if our stream stops,
# flight_interface holds a point on the path, never one inside a wall (sim_streamer.py).

PX4_V116_MPC_XY_P = 0.95


@dataclass
class Setpoint:
    tau: float             # path time of the on-path point
    rate: float            # dtau/dt (path s per wall s), already including the time scale
    p: np.ndarray          # (3,) on-path point p(tau): where the vehicle should be now; HOLD target
    v: np.ndarray          # (3,) velocity feedforward dp/dt = v(tau) rate (only with a FF interface)
    a: np.ndarray          # (3,) acceleration feedforward d2p/dt2
    q: np.ndarray          # (3,) lead point p(tau + rate L): what the position-only stream sends
    yaw: float             # course heading at the lead point
    lag: float             # error the governor uses: cross-track + behind, the ahead part ignored, m
    err: float             # |vehicle - p(tau)|, m


class PathClock:
    """Path time tau runs at `scale` x wall time, slowed smoothly when the vehicle falls behind.

    dtau/dt = scale * r, with r in [0, 1] rate-limited to `r_slew` per s (so it starts from rest),
    chasing r* = clip((e_stop - lag) / (e_stop - e_ok), 0, 1).
    lag = |vehicle - p(tau)| with any component AHEAD of p(tau) along the path tangent removed: an
    overshoot does not slow the clock (P-control pulls it back while tau moves on).
    Sent: q = p(tau + dtau/dt * lead_s), lead_s = 1 / MPC_XY_P. The lead shrinks with the rate, so
    when the clock stops the setpoint comes back to p(tau) smoothly.
    Feedforward (for an interface that carries it): v = v(tau) rate, a = a(tau) rate^2 + v(tau) dr/dt scale."""

    def __init__(self, path: CoursePath, scale=1.0, lead_s=1.0 / PX4_V116_MPC_XY_P, e_ok_m=0.3,
                 e_stop_m=0.6, r_slew=1.0):
        self.path, self.scale, self.lead = path, float(scale), float(lead_s)
        self.e_ok, self.e_stop, self.r_slew = float(e_ok_m), float(e_stop_m), float(r_slew)
        self.tau = 0.0
        self.r = 0.0
        self.stalled = False   # r == 0: the clock is stopped, waiting for the vehicle

    @property
    def done(self):
        return self.tau >= self.path.duration

    @staticmethod
    def lag_error(vehicle_xyz, p, v):
        d = np.asarray(vehicle_xyz, float) - p
        n = float(np.linalg.norm(v))
        if n < 1e-3:
            return float(np.linalg.norm(d))
        along = float(d @ v) / n
        return math.sqrt(max(float(d @ d) - along * along, 0.0) + min(along, 0.0) ** 2)

    def step(self, dt, vehicle_xyz_course):
        dt = float(dt)
        p, v, _, _ = self.path.state(self.tau)
        lag = self.lag_error(vehicle_xyz_course, p, v)
        r_target = min(max((self.e_stop - lag) / (self.e_stop - self.e_ok), 0.0), 1.0)
        dr = min(max(r_target - self.r, -self.r_slew * dt), self.r_slew * dt)
        self.r += dr
        self.stalled = self.r <= 0.0
        self.tau = min(self.tau + self.scale * self.r * dt, self.path.duration)
        p, v, a, _ = self.path.state(self.tau)
        rate = 0.0 if self.done else self.scale * self.r
        v_ff = v * rate
        a_ff = a * rate ** 2 + v * self.scale * (dr / dt if dt > 0 and not self.done else 0.0)
        xy, z, yaw = self.path.at(self.tau + rate * self.lead)
        err = float(np.linalg.norm(np.asarray(vehicle_xyz_course, float) - p))
        return Setpoint(self.tau, rate, p, v_ff, a_ff, np.r_[xy, z], yaw, lag, err)


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

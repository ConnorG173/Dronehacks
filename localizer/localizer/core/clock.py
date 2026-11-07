"""PathClock: governed path time plus the on-path lead point. Pure numpy, course frame.

PX4 v1.16 Offboard has no waypoint logic: mc_pos_control (PositionControl.cpp) tracks whatever
TrajectorySetpoint is current, with
  v_sp = Kp (p_sp - x) + v_ff                    Kp = MPC_XY_P 0.95, MPC_Z_P 1.0 (defaults)
  a_sp = Kv (v_sp - v) + I - D dv/dt + a_ff      Kv = MPC_XY_VEL_P_ACC 1.8
A position-only interface (the real drone's flight_interface: v_ff = a_ff = NaN) makes a setpoint
ON the vehicle lag by v / Kp (1.05 s of travel). Sending the path point a time L = 1 / Kp ahead gives
  Kp (p(tau + L) - x) = Kp (p(tau) - x) + v + a / (2 Kp) + ...
i.e. velocity feedforward plus roughly the acceleration feedforward (a / 1.80 vs the ideal
a / Kv = a / 1.8 through the velocity loop). Unlike the algebraic p + v/Kp + a/(Kp Kv), the
lead stays a point of the planned, collision-checked path: if our stream stops,
flight_interface holds a point on the path, never one inside a wall.
An interface that carries feedforward (the simulator's TrajectorySetpoint) sends p, v, a instead.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .path import CoursePath

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

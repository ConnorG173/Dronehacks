"""Velocity-setpoint trajectory tracker. No ROS; all quantities in map ENU.

The plant is PX4's velocity loop behind the organizers' flight interface: velocity
(and yaw rate) only, zero-order held at ``setpoint_rate_hz``, norm-clamped to
``max_speed_mps``. Approximately a first-order lag (tau_v ~ 1/MPC_XY_VEL_P_ACC) plus
dead time. See docs/proposals/path_following.md section 3.2.

Control law per step, path clock tau with rate taudot:
  v_ff  = v_ref(tau + t_lead * taudot) * taudot      (lag compensation by look-ahead)
  e     = p_ref(tau) - p ; split into along-track / cross-track / vertical
  v_cmd = v_ff + Ka e_a + Kc e_c + Kz e_z (+ I)      saturated with correction priority
  taudot follows a law that slows the clock when the vehicle is off or behind the path.
"""
import math
from dataclasses import dataclass

import numpy as np

from .frames import wrap_pi


@dataclass
class TrackerParams:
    kc: float = 1.2           # 1/s cross-track gain
    ka: float = 0.6           # 1/s along-track gain
    kz: float = 1.5           # 1/s vertical gain
    ki: float = 0.0           # 1/s^2 integral gain on cross-track + vertical error
    i_max: float = 0.3        # m/s integral clamp
    ff_mode: str = 'inverse'  # 'lookahead': v(tau + t_lead) | 'inverse': v + tau_v a at tau + t_delay
    t_lead: float = 0.6       # s look-ahead on the feedforward velocity (lookahead mode)
    tau_v: float = 0.55       # s PX4 velocity-loop time constant (inverse mode)
    t_delay: float = 0.10     # s dead time: zero-order hold + VIO/link latency (inverse mode)
    v_max: float = 1.4        # m/s norm budget (interface cap minus margin)
    e0: float = 0.10          # m error with no slow-down
    e_span: float = 0.30      # m extra error that stops the clock
    taudot_max: float = 1.0   # max path-clock rate (1 = planned timing)
    taudot_acc: float = 1.0   # 1/s, max change of taudot per second
    k_yaw: float = 1.5        # 1/s yaw gain
    yaw_rate_max: float = math.radians(90.0)
    yaw_mode: str = 'path'    # 'path' | 'fixed'
    z_ceiling: float = 1.6    # m (map z), enforced here: twists are not altitude-clamped
    z_floor: float = 0.25     # m (map z), during the course only
    guard_margin: float = 0.1  # m


def _cap_norm(v, vmax):
    n = float(np.linalg.norm(v))
    return v * (vmax / n) if n > vmax > 0 else v


def priority_saturate(v_corr, v_along, vmax):
    """|v_corr + alpha v_along| <= vmax with alpha in [0, 1] as large as possible.
    Correction first; if it alone exceeds the budget it is scaled and alpha = 0."""
    if np.linalg.norm(v_corr) >= vmax:
        return _cap_norm(v_corr, vmax), 0.0
    total = v_corr + v_along
    if np.linalg.norm(total) <= vmax:
        return total, 1.0
    # |c + a u|^2 = vmax^2  ->  a^2 u.u + 2 a c.u + c.c - vmax^2 = 0, take the positive root
    uu = float(v_along @ v_along)
    cu = float(v_corr @ v_along)
    cc = float(v_corr @ v_corr)
    disc = cu * cu - uu * (cc - vmax * vmax)
    alpha = (-cu + math.sqrt(max(disc, 0.0))) / uu if uu > 0 else 0.0
    alpha = min(max(alpha, 0.0), 1.0)
    return v_corr + alpha * v_along, alpha


class Tracker:

    def __init__(self, traj, params=None):
        self.traj = traj
        self.p = params or TrackerParams()
        self.reset()

    def reset(self, tau=0.0, taudot=0.0):
        self.tau = float(tau)
        self.taudot = float(taudot)
        self.integral = np.zeros(3)
        self.fixed_yaw = None
        self.last = {}

    @property
    def finished(self):
        return self.tau >= self.traj.duration - 1e-9

    def clock_target(self, e_cross, e_behind, speed_scale=1.0):
        p = self.p
        slow = max(0.0, e_cross - p.e0) / p.e_span + max(0.0, e_behind - p.e0) / p.e_span
        return min(max(1.0 - slow, 0.0), 1.0) * p.taudot_max * min(max(speed_scale, 0.0), 1.0)

    def step(self, dt, pos, yaw, speed_scale=1.0):
        """One control step.

        pos, yaw     current vehicle position (map) and heading (map, rad)
        speed_scale  0..1 extra slow-down (localization status, clearance)
        returns (v_cmd map ENU (3,), yaw_rate_cmd rad/s)
        """
        p = self.p
        tr = self.traj
        pos = np.asarray(pos, float)

        p_ref, _, _, yaw_ref, yaw_rate_ref = tr.sample(self.tau)
        t_hat = tr.tangent(self.tau)
        e = p_ref - pos
        e_z = np.array([0.0, 0.0, e[2]])
        e_h = e - e_z
        t_h = np.array([t_hat[0], t_hat[1], 0.0])
        n = np.linalg.norm(t_h)
        t_h = t_h / n if n > 1e-6 else t_h
        e_a = (e_h @ t_h) * t_h
        e_c = e_h - e_a
        e_behind = float(e_h @ t_h)   # > 0: vehicle behind the reference

        # path clock
        target = self.clock_target(float(np.linalg.norm(e_c)), e_behind, speed_scale)
        step = p.taudot_acc * dt
        self.taudot += min(max(target - self.taudot, -step), step)
        self.tau = min(self.tau + self.taudot * dt, tr.duration)

        # feedforward, scaled by the clock rate (chain rule: d/dt p(tau) = v taudot)
        if p.ff_mode == 'inverse':
            # invert a first-order lag with dead time: u(t) = v_r(t + d) + tau_v a_r(t + d)
            _, v_d, a_d, _, _ = tr.sample(self.tau + p.t_delay * self.taudot)
            v_ff = v_d * self.taudot + p.tau_v * a_d * self.taudot ** 2
        else:
            _, v_look, _, _, _ = tr.sample(self.tau + p.t_lead * self.taudot)
            v_ff = v_look * self.taudot

        # feedback
        if p.ki > 0.0:
            self.integral = _cap_norm(self.integral + p.ki * (e_c + e_z) * dt, p.i_max)
        v_corr = p.kc * e_c + p.kz * e_z + self.integral
        v_along = v_ff + p.ka * e_a
        v_cmd, alpha = priority_saturate(v_corr, v_along, p.v_max)

        # altitude guards (the flight interface does not clamp twists)
        z = pos[2]
        if z > p.z_ceiling - p.guard_margin:
            v_cmd[2] = min(v_cmd[2], -p.kz * (z - p.z_ceiling + p.guard_margin))
        if z < p.z_floor:
            v_cmd[2] = max(v_cmd[2], p.kz * (p.z_floor - z))
        v_cmd = _cap_norm(v_cmd, p.v_max)

        # yaw
        if p.yaw_mode == 'fixed':
            if self.fixed_yaw is None:
                self.fixed_yaw = float(tr.yaw[0])
            yaw_ref, yaw_rate_ref = self.fixed_yaw, 0.0
        yaw_rate = yaw_rate_ref * self.taudot + p.k_yaw * float(wrap_pi(yaw_ref - yaw))
        yaw_rate = min(max(yaw_rate, -p.yaw_rate_max), p.yaw_rate_max)

        self.last = dict(tau=self.tau, taudot=self.taudot, e_along=e_behind,
                         e_cross=float(np.linalg.norm(e_c)), e_z=float(e[2]),
                         alpha=alpha, v_ff=v_ff, v_cmd=v_cmd.copy(), p_ref=p_ref)
        return v_cmd, yaw_rate


def hold_velocity(pos, target, kp=1.0, v_xy_max=0.5, v_z_max=0.6, kp_z=None):
    """Simple point-hold law for takeoff/landing phases."""
    e = np.asarray(target, float) - np.asarray(pos, float)
    v = kp * e
    if kp_z is not None:
        v[2] = kp_z * e[2]
    v[:2] = _cap_norm(v[:2], v_xy_max)
    v[2] = min(max(v[2], -v_z_max), v_z_max)
    return v

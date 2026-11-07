"""A small model of PX4 v1.16 mc_pos_control in Offboard, for offline tests of the setpoint stream.

From PositionControl.cpp / ControlMath.cpp (tag v1.16.2):
  _positionControl:  v_sp = Kp (p_sp - x) added to the velocity feedforward (NaN skipped); xy saturated
                     with constrainXY (the position term has priority over the feedforward), z clamped
                     to [-MPC_Z_VEL_MAX_UP, MPC_Z_VEL_MAX_DN] (NED).
  _velocityControl:  a_sp = Kv e_v + I - Kd dv/dt (D on the measured acceleration) + acceleration feedforward.
  _accelerationControl: tilt limited to MPC_TILTMAX_AIR. No MPC_ACC_HOR / jerk limit in Offboard (those
                     only feed GotoControl / flight tasks).
Defaults: MPC_XY_P 0.95, MPC_Z_P 1.0, MPC_XY_VEL_P/I/D_ACC 1.8/0.4/0.2, MPC_Z_VEL_P_ACC 4.0,
MPC_XY_VEL_MAX 12, MPC_Z_VEL_MAX_UP 3.0, MPC_Z_VEL_MAX_DN 1.5, MPC_TILTMAX_AIR 45 deg.

Modelling assumptions (not PX4 parameters): MPC_Z_VEL_I_ACC 2.0 and D 0 (not checked), no thrust
saturation, no drag, perfect state estimate, and a first-order lag `tau_att_s` between the
acceleration setpoint and the achieved acceleration (attitude loop + motors). Frame: course x, y,
z up (the z limits are mapped accordingly).
"""
import math

import numpy as np

G = 9.81


def constrain_xy(v0, v1, vmax):
    """ControlMath::constrainXY: v0 + v1 limited to vmax, keeping v0 (the P term) first."""
    s = v0 + v1
    if np.linalg.norm(s) <= vmax:
        return s
    n0 = float(np.linalg.norm(v0))
    if n0 >= vmax:
        return v0 / n0 * vmax
    u1 = v1 / np.linalg.norm(v1)
    # |v0 + m u1| = vmax, m >= 0
    b = float(v0 @ u1)
    m = -b + math.sqrt(b * b - (n0 * n0 - vmax * vmax))
    return v0 + m * u1


class PX4PosModel:
    def __init__(self, x0, kp_xy=0.95, kp_z=1.0, kv_xy=1.8, ki_xy=0.4, kd_xy=0.2,
                 kv_z=4.0, ki_z=2.0, kd_z=0.0, vxy_max=12.0, vz_up=3.0, vz_dn=1.5,
                 tilt_deg=45.0, tau_att_s=0.1):
        self.x = np.asarray(x0, float).copy()
        self.v = np.zeros(3)
        self.a = np.zeros(3)               # achieved acceleration
        self.i = np.zeros(3)
        self.kp = np.array([kp_xy, kp_xy, kp_z])
        self.kv = np.array([kv_xy, kv_xy, kv_z])
        self.ki = np.array([ki_xy, ki_xy, ki_z])
        self.kd = np.array([kd_xy, kd_xy, kd_z])
        self.vxy_max, self.vz_up, self.vz_dn = vxy_max, vz_up, vz_dn
        self.axy_max = G * math.tan(math.radians(tilt_deg))
        self.tau = tau_att_s

    def step(self, dt, pos_sp, vel_ff=None, acc_ff=None, push=None):
        """One controller + plant step. vel_ff / acc_ff None = NaN in TrajectorySetpoint.
        `push` is an external acceleration (a gust), m/s^2."""
        vff = np.zeros(3) if vel_ff is None else np.asarray(vel_ff, float)
        aff = np.zeros(3) if acc_ff is None else np.asarray(acc_ff, float)
        vp = (np.asarray(pos_sp, float) - self.x) * self.kp
        v_sp = np.empty(3)
        v_sp[:2] = constrain_xy(vp[:2], vff[:2], self.vxy_max)
        v_sp[2] = min(max(vp[2] + vff[2], -self.vz_dn), self.vz_up)
        e = v_sp - self.v
        self.i += e * self.ki * dt
        a_sp = e * self.kv + self.i - self.kd * self.a + aff
        n = float(np.linalg.norm(a_sp[:2]))
        if n > self.axy_max:
            a_sp[:2] *= self.axy_max / n
        self.a += (a_sp - self.a) * min(dt / self.tau, 1.0)
        acc = self.a + (0 if push is None else np.asarray(push, float))
        self.v += acc * dt
        self.x += self.v * dt
        return self.x

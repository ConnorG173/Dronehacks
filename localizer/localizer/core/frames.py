"""Frames, transforms, attitude helpers and the course fence. Pure numpy.

Frames:
  course  planner map frame: x, y metres, z up = height above the floor, yaw CCW from +x
  local   the vehicle's own estimate frame, written as ENU with flight_interface's convention:
          x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned, yaw_enu = pi/2 - yaw_ned.
          real drone: cuVSLAM odom xy (x forward at stack start) + PX4 local height
          simulator:  PX4 vehicle_local_position (NED) converted with the convention above
  T       course <- local, (x, y, yaw): p_course = R(T.yaw) p_local + (T.x, T.y).
          Height is never rotated: z_course = z_local - z0 (z0 chosen by the follower).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rot2(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


# ----------------------------------------------------------------------------- NED <-> ENU

def ned_to_enu(v):
    """Vector or point (x, y, z) NED -> ENU (flight_interface convention). Its own inverse."""
    v = np.asarray(v, float)
    return np.array([v[1], v[0], -v[2]])


enu_to_ned = ned_to_enu


def yaw_ned_to_enu(yaw_ned):
    """Heading clockwise from NED +x -> CCW from ENU +x. Its own inverse."""
    return wrap(math.pi / 2 - yaw_ned)


yaw_enu_to_ned = yaw_ned_to_enu


# ----------------------------------------------------------------------------- attitude

def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


_NED_ENU = np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, -1]])    # its own inverse
_FRD_FLU = np.diag([1.0, -1, -1])                            # its own inverse


def px4_to_enu_flu(q_wxyz):
    """PX4 VehicleAttitude.q (w, x, y, z; FRD body -> NED) -> rotation FLU body -> ENU."""
    w, x, y, z = (float(v) for v in q_wxyz)
    return _NED_ENU @ quat_to_mat(x, y, z, w) @ _FRD_FLU


def tilt_deg(R_world_body):
    """Angle between body z and world z (for the pad check: odom tilt vs PX4 tilt)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R_world_body[2, 2])))))


def yaw_of(R_world_body):
    return math.atan2(R_world_body[1, 0], R_world_body[0, 0])


def yaw_quat_xyzw(yaw):
    """Quaternion (x, y, z, w) of a pure rotation by yaw about +z."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


# ----------------------------------------------------------------------------- T = course <- local

@dataclass
class Transform2D:
    x: float
    y: float
    yaw: float

    def to_course(self, xy_local, yaw_local=None):
        p = rot2(self.yaw) @ np.asarray(xy_local, float)[:2] + (self.x, self.y)
        return p if yaw_local is None else (p, wrap(yaw_local + self.yaw))

    def to_local(self, xy_course, yaw_course=None):
        p = rot2(-self.yaw) @ (np.asarray(xy_course, float)[:2] - (self.x, self.y))
        return p if yaw_course is None else (p, wrap(yaw_course - self.yaw))

    def vec_to_local(self, v_course):
        """Rotate a course (x, y, z) vector into the local frame (z unchanged)."""
        v = np.asarray(v_course, float)
        return np.r_[rot2(-self.yaw) @ v[:2], v[2]]


def pad_init(local_xy, local_yaw, local_tilt_deg, imu_tilt_deg, pad_xy, pad_yaw,
             max_offset_m=0.15, max_yaw_deg=5.0, max_tilt_diff_deg=3.0):
    """T0 = pad pose o local pose^-1, or (None, reason).

    Assumes the vehicle sits on the launch mark facing the pad heading (HALF-ASSED: nothing
    measures that; the brief's wide registration cross-check against the CAD is not done).
    A limit of None switches that check off. The tilt check compares the local estimate's tilt
    with the IMU's and needs both (None = not available yet -> refuse)."""
    off = float(np.hypot(*np.asarray(local_xy, float)[:2]))
    if max_offset_m is not None and off > max_offset_m:
        return None, f"local position not near zero ({off:.2f} m): restart the stack on the pad"
    if max_yaw_deg is not None and abs(math.degrees(wrap(local_yaw))) > max_yaw_deg:
        return None, f"local yaw {math.degrees(wrap(local_yaw)):.1f} deg not near zero"
    if max_tilt_diff_deg is not None:
        if local_tilt_deg is None or imu_tilt_deg is None:
            return None, "waiting for odometry and IMU attitude (tilt check)"
        if abs(local_tilt_deg - imu_tilt_deg) > max_tilt_diff_deg:
            return None, (f"odom tilt {local_tilt_deg:.1f} deg vs PX4 {imu_tilt_deg:.1f} deg: "
                          f"the visual odometry attitude disagrees with the IMU")
    yaw = wrap(pad_yaw - local_yaw)
    t = np.asarray(pad_xy, float) - rot2(yaw) @ np.asarray(local_xy, float)[:2]
    return Transform2D(float(t[0]), float(t[1]), yaw), "ok"


# ----------------------------------------------------------------------------- fence

def inside_fence(xy_course, cage_half_m=3.049, margin_m=0.3):
    lim = cage_half_m - margin_m
    return bool(np.all(np.abs(np.asarray(xy_course, float)[:2]) <= lim))


def clamp_to_fence(xy_course, cage_half_m=3.049, margin_m=0.3):
    lim = cage_half_m - margin_m
    return np.clip(np.asarray(xy_course, float)[:2], -lim, lim)

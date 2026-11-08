"""Frame conversions for path following. No ROS.

planner NED  : planner/README.md "Frames": origin StartPos (pad, ground), +x along
               StartHeadingDeg, +y right of it, +z down.
map (ENU)    : course frame, z up (docs/CONTRACTS.md).
odom (ENU)   : cuVSLAM's frame. map -> odom is a planar rigid transform plus a z
               offset (x, y, z, yaw): VIO has no gravity drift in roll/pitch that we
               correct here.
"""
import math

import numpy as np


def wrap_pi(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def _ned_rot(psi0):
    c, s = math.cos(psi0), math.sin(psi0)
    # symmetric and its own inverse (a reflection composed with a rotation)
    return np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])


def map_to_planner_ned(p_map, start_pos, psi0):
    """The planner's own forward conversion (for tests)."""
    return (np.asarray(p_map) - start_pos) @ _ned_rot(psi0).T


def planner_ned_to_map_points(p_ned, start_pos, psi0):
    return np.asarray(p_ned) @ _ned_rot(psi0).T + np.asarray(start_pos)


def planner_ned_to_map_vectors(v_ned, psi0):
    return np.asarray(v_ned) @ _ned_rot(psi0).T


def planner_yaw_to_map(yaw_ned, psi0):
    """yaw_ned = psi0 - theta  =>  theta = psi0 - yaw_ned."""
    return wrap_pi(psi0 - np.asarray(yaw_ned))


class Pose2:
    """Planar pose (x, y, z, yaw): the transform T_a_b taking b-coordinates to a."""

    __slots__ = ('x', 'y', 'z', 'yaw')

    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.x, self.y, self.z, self.yaw = float(x), float(y), float(z), float(yaw)

    def __repr__(self):
        return f'Pose2({self.x:.3f}, {self.y:.3f}, {self.z:.3f}, {math.degrees(self.yaw):.1f}deg)'

    @property
    def t(self):
        return np.array([self.x, self.y, self.z])

    def rot(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def apply(self, p):
        """b-point -> a-point."""
        return self.rot() @ np.asarray(p, dtype=float) + self.t

    def apply_vec(self, v):
        return self.rot() @ np.asarray(v, dtype=float)

    def inverse(self):
        r = self.rot().T
        t = -r @ self.t
        return Pose2(t[0], t[1], t[2], -self.yaw)

    def compose(self, other):
        """self * other  (T_a_b * T_b_c = T_a_c)."""
        t = self.apply(other.t)
        return Pose2(t[0], t[1], t[2], float(wrap_pi(self.yaw + other.yaw)))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def map_from_launch(course):
    """T_map_base at launch: the drone sitting on StartPos facing StartHeadingDeg."""
    s = course.start_pos
    return Pose2(s[0], s[1], s[2], course.start_heading_rad)


def map_odom_at_go(t_map_base_launch, t_odom_base_now):
    """Fallback map->odom (contract): assume the drone is on the launch pose now.
    T_map_odom = T_map_base * T_odom_base^-1."""
    return t_map_base_launch.compose(t_odom_base_now.inverse())

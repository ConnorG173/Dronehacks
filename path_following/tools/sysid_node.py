#!/usr/bin/env python3
"""System-identification flight: velocity steps through the flight interface, for fit_lag.py.

Twists only (like mission_tracker). Sequence, after the pilot's go (offboard_active):
  climb over the launch point to --height, move to the centre of the course MapBound, hover 2 s,
  then steps: +x, 0, -x, 0, +y, 0, -y, 0, diagonal +, 0, diagonal -, 0 (amplitude --amp, --step-s each),
  then return over the launch point and descend until PX4 reports landed.
Publishes /team/tracker/debug with the same layout as mission_tracker (state 2 during the steps),
so fit_lag.py works on the bag unchanged. Height/centre are derived from the course config; pick a
height above every obstacle (sim: above the 1.73 m walls, below the net). NOT flight code for runs.

    python3 sysid_node.py COURSE.yaml --pose-topic /sim/ground_truth/odom --height 2.4 --amp 0.8
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'team_path_following'))
from team_path_following import frames  # noqa: E402
from team_path_following.course import load_course  # noqa: E402
from team_path_following.node import DEBUG_FIELDS  # noqa: E402

try:
    from px4_msgs.msg import VehicleLocalPosition
except ImportError:  # pragma: no cover
    VehicleLocalPosition = None


class SysId(Node):

    def __init__(self, a):
        super().__init__('sysid')
        self.a = a
        c = load_course(a.course)
        b = c.map_bound
        self.launch = np.array([c.start_pos[0], c.start_pos[1], a.height])
        self.centre = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, a.height])
        self.T = frames.map_from_launch(c)        # course <- odom (fallback, as mission_tracker)
        self.pad_z = float(c.start_pos[2])
        self.pos = self.vel = None
        self.px4_vel = None
        self.state = {}
        s = a.amp
        d = s / math.sqrt(2)
        unit = {'x': [(s, 0), (0, 0), (-s, 0), (0, 0)], 'y': [(0, s), (0, 0), (0, -s), (0, 0)],
                'd': [(d, d), (0, 0), (-d, -d), (0, 0)]}
        self.steps = [st for _ in range(a.repeat) for ax in a.axes.split(',') for st in unit[ax]]
        if a.centre == 'launch':
            self.centre = self.launch.copy()
        self.centre[:2] += np.array(a.offset)
        self.phase, self.t_phase, self.k = 'wait', None, 0
        self.create_subscription(Odometry, a.pose_topic, self.pose_cb, 10)
        self.create_subscription(String, '/skybeta/state', lambda m: self.state.update(json.loads(m.data)), 10)
        if VehicleLocalPosition is not None and a.px4_velocity:
            from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
            q = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.px4_cb, q)
        self.pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.dbg = self.create_publisher(Float64MultiArray, '/team/tracker/debug', 10)
        self.mstate = self.create_publisher(String, '/team/mission/state', 10)
        self.create_timer(0.02, self.tick)

    def pose_cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.pos = np.array([p.x, p.y, p.z])
        yaw = frames.yaw_from_quat(q.x, q.y, q.z, q.w)
        t = m.twist.twist.linear
        c, s = math.cos(yaw), math.sin(yaw)
        self.vel = np.array([c * t.x - s * t.y, s * t.x + c * t.y, t.z])   # body -> course (small tilt)

    def px4_cb(self, m):
        if m.v_xy_valid:
            self.px4_vel = self.T.apply_vec(np.array([m.vy, m.vx, -m.vz]))

    def goto(self, target, vmax=0.6):
        e = target - self.pos
        v = 1.2 * e
        n = np.linalg.norm(v)
        return v * (vmax / n) if n > vmax else v, float(np.linalg.norm(e))

    def tick(self):
        now = time.monotonic()
        u = np.zeros(3)
        st = 0
        if self.pos is None:
            return
        off = bool(self.state.get('offboard_active'))
        if self.phase == 'wait':
            if off:
                self.phase, self.t_phase = 'climb', now
        elif not off and self.phase not in ('done',):
            self.phase = 'done'
            self.get_logger().warn('offboard ended')
        elif self.phase == 'climb':
            st = 1
            tgt = np.array([self.launch[0], self.launch[1], self.a.height])
            u, e = self.goto(tgt, 0.8)
            if e < 0.1:
                self.phase = 'to_centre'
        elif self.phase == 'to_centre':
            st = 1
            u, e = self.goto(self.centre, 0.6)
            if e < 0.08:
                self.phase, self.t_phase = 'hover', now
        elif self.phase == 'hover':
            st = 1
            u, _ = self.goto(self.centre)
            if now - self.t_phase > 2.0:
                self.phase, self.t_phase, self.k = 'steps', now, 0
        elif self.phase == 'steps':
            st = 2
            vx, vy = self.steps[self.k]
            u = np.array([vx, vy, 0.0])
            u[2] = 1.5 * (self.a.height - self.pos[2])
            if now - self.t_phase > self.a.step_s:
                self.k += 1
                self.t_phase = now
                if self.k >= len(self.steps):
                    self.phase = 'back'
        elif self.phase == 'back':
            st = 3
            u, e = self.goto(self.launch, 0.6)
            if e < 0.1:
                self.phase = 'descend'
        elif self.phase == 'descend':
            st = 3
            u, _ = self.goto(np.array([self.launch[0], self.launch[1], self.pad_z]), 0.3)
            u[2] = -0.5 if self.pos[2] - self.pad_z > 0.4 else -0.25
            if self.state.get('landed'):
                self.phase = 'landed'
        elif self.phase == 'landed':
            st = 4
            u = np.array([0.0, 0.0, -0.25])
            if not self.state.get('armed', True):
                self.phase = 'done'
        if self.phase == 'done':
            st = 5
            u = np.zeros(3)
        v_odom = self.T.inverse().apply_vec(u)
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = (float(x) for x in v_odom)
        self.pub.publish(m)   # zero twists while waiting keep auto-offboard eligible
        vel = self.px4_vel if self.px4_vel is not None else (self.vel if self.vel is not None else np.full(3, math.nan))
        row = {k: math.nan for k in DEBUG_FIELDS}
        row.update(t=now, state=st, vcmd_x=u[0], vcmd_y=u[1], vcmd_z=u[2], v_x=vel[0], v_y=vel[1], v_z=vel[2],
                   p_x=self.pos[0], p_y=self.pos[1], p_z=self.pos[2], pose_src=1)
        d = Float64MultiArray()
        d.layout.dim.append(MultiArrayDimension(label=','.join(DEBUG_FIELDS), size=len(DEBUG_FIELDS),
                                                stride=len(DEBUG_FIELDS)))
        d.data = [float(row[k]) for k in DEBUG_FIELDS]
        self.dbg.publish(d)
        self.mstate.publish(String(data={'wait': 'idle', 'climb': 'takeoff', 'to_centre': 'takeoff',
                                         'hover': 'takeoff', 'steps': 'course', 'back': 'landing',
                                         'descend': 'landing', 'landed': 'landed', 'done': 'done'}[self.phase]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('course')
    ap.add_argument('--pose-topic', default='/sim/ground_truth/odom')
    ap.add_argument('--height', type=float, required=True)
    ap.add_argument('--amp', type=float, default=0.8)
    ap.add_argument('--step-s', type=float, default=1.5)
    ap.add_argument('--axes', default='x,y,d', help='step axes, e.g. y')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--centre', choices=['bound', 'launch'], default='bound',
                    help='step around the MapBound centre or the launch point (+ --offset)')
    ap.add_argument('--offset', type=float, nargs=2, default=[0.0, 0.0])
    ap.add_argument('--px4-velocity', action='store_true', help='measure with EKF2 velocity instead of the pose twist')
    a, rest = ap.parse_known_args()
    rclpy.init(args=rest)
    n = SysId(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

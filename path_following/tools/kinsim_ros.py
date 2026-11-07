#!/usr/bin/env python3
"""ROS 2 stand-in for flight interface + PX4 + VIO (offline end-to-end test of the
real mission_tracker node, no Gazebo/PX4 needed).

Subscribes /skybeta/setpoint/twist. Publishes /visual_slam/tracking/odometry (odom
frame = launch pose), /sim/ground_truth/odom (map), /skybeta/state (JSON like the
flight interface). "Pilot": arms at t=1 s and gives the go at t=2 s if a team setpoint
arrived within the last second (auto_offboard rule).

    python3 kinsim_ros.py COURSE.yaml [--setpoint-rate 10] [--exit-when-done]
"""
import argparse
import json
import math
import os
import time
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'team_path_following'))
from kinsim import Px4Model, SimParams  # noqa: E402
from team_path_following import frames  # noqa: E402
from team_path_following.course import load_course  # noqa: E402


def odom_msg(frame, p, yaw, v_world, stamp):
    m = Odometry()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.child_frame_id = 'base_link'
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = (float(c) for c in p)
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    c, s = math.cos(yaw), math.sin(yaw)
    vb = (c * v_world[0] + s * v_world[1], -s * v_world[0] + c * v_world[1], v_world[2])
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = (float(x) for x in vb)
    return m


class KinSimRos(Node):

    def __init__(self, course, sp, exit_when_done):
        super().__init__('kinsim')
        self.sp = sp
        self.course = course
        self.T = frames.map_from_launch(course)   # map <- odom
        self.px4 = Px4Model(sp, course.start_pos, course.start_heading_rad)
        self.px4.armed = False
        self.rng = np.random.default_rng(sp.seed)
        self.twist = None        # (rx_time, v_odom, yaw_rate)
        self.px4_sp = (np.zeros(3), 0.0)
        self.offboard = False
        self.went = False
        self._wall = None
        self._acc = 0.0
        self.t = 0.0
        self.t_first_twist = None
        self.t_armed = None
        self.odom_q = []
        self.exit_when_done = exit_when_done
        self.create_subscription(TwistStamped, '/skybeta/setpoint/twist', self.twist_cb, 10)
        self.vio_pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        self.gt_pub = self.create_publisher(Odometry, '/sim/ground_truth/odom', 10)
        self.state_pub = self.create_publisher(String, '/skybeta/state', 10)
        self.create_subscription(String, '/team/mission/state', self.mission_cb, 10)
        self.mission_state = ''
        self.create_timer(0.01, self.physics)
        self.create_timer(1.0 / sp.setpoint_rate_hz, self.fi_tick)
        self.create_timer(1.0 / sp.odom_hz, self.vio)
        self.create_timer(0.2, self.state)

    def mission_cb(self, msg):
        if msg.data != self.mission_state:
            self.get_logger().info(f'mission state: {msg.data} at t={self.t:.2f}')
        self.mission_state = msg.data
        if self.exit_when_done and msg.data in ('done', 'abort'):
            p = self.px4.p
            self.get_logger().info(f'END {msg.data}: pos {p.round(3)}, goal {self.course.goal.round(3)}')
            raise SystemExit(0)

    def twist_cb(self, msg):
        v = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        if np.all(np.isfinite(v)):
            n = np.linalg.norm(v)
            if n > self.sp.max_speed_mps:
                v *= self.sp.max_speed_mps / n
            self.twist = (self.t, v, msg.twist.angular.z)

    def fi_tick(self):
        if self.twist is not None and self.t - self.twist[0] < self.sp.twist_timeout_s:
            self.px4_sp = (self.T.apply_vec(self.twist[1]), self.twist[2])
        else:
            self.px4_sp = (-0.5 * self.px4.v, 0.0)

    def physics(self):
        # step by real elapsed time so the plant runs at wall-clock speed like the node
        now = time.monotonic()
        elapsed = min(now - self._wall, 0.05) if self._wall is not None else 0.01
        self._wall = now
        self._acc += elapsed
        while self._acc >= self.sp.dt:
            self._acc -= self.sp.dt
            self.t += self.sp.dt
            streaming = self.twist is not None and self.t - self.twist[0] < 0.5
            if streaming and self.t_first_twist is None:
                self.t_first_twist = self.t
            # pilot: arms once the team node has been streaming for 1 s, switches 1 s later
            if not self.px4.armed and not self.went and self.t_first_twist is not None \
                    and self.t - self.t_first_twist > 1.0:
                self.px4.armed = True
                self.t_armed = self.t
                self.get_logger().info('pilot: armed')
            if self.px4.armed and not self.went and self.t_armed is not None and self.t - self.t_armed > 1.0:
                self.went = True
                if self.twist is not None and self.t - self.twist[0] < 1.0:
                    self.offboard = True
                    self.get_logger().info('go: Offboard')
                else:
                    self.get_logger().warn('go refused: no team setpoint in the last second')
            if self.px4.armed and not self.px4.flew and self.t_armed is not None and self.t - self.t_armed > 10.0:
                self.px4.armed = False   # COM_DISARM_PRFLT
                self.get_logger().warn('disarmed: no takeoff within 10 s')
            v, yr = self.px4_sp if self.offboard else (np.zeros(3), 0.0)
            self.px4.step(self.sp.dt, v, yr, self.t)
        if not self.px4.armed:
            self.offboard = False
        if self.exit_when_done and self.t > 120.0:
            self.get_logger().error('END timeout')
            raise SystemExit(1)
        stamp = self.get_clock().now().to_msg()
        self.gt_pub.publish(odom_msg('map', self.px4.p, self.px4.yaw, self.px4.v, stamp))

    def vio(self):
        Ti = self.T.inverse()
        p = Ti.apply(self.px4.p) + self.rng.normal(0, self.sp.odom_noise_m, 3)
        yaw = float(frames.wrap_pi(self.px4.yaw - self.T.yaw))
        msg = odom_msg('odom', p, yaw, Ti.apply_vec(self.px4.v), self.get_clock().now().to_msg())
        self.odom_q.append((self.t + self.sp.odom_latency_s, msg))
        while self.odom_q and self.odom_q[0][0] <= self.t:
            self.vio_pub.publish(self.odom_q.pop(0)[1])

    def state(self):
        Ti = self.T.inverse()
        s = {'stamp': self.t, 'px4_link': True, 'armed': self.px4.armed, 'landed': self.px4.landed,
             'nav_state': 14 if self.offboard else 2,
             'nav_state_name': 'OFFBOARD' if self.offboard else 'POSCTL',
             'offboard_active': self.offboard, 'odom_enu': list(Ti.apply(self.px4.p)),
             'setpoint_mode': 'velocity' if self.twist else 'hold', 'clamped': [], 'frozen': False}
        self.state_pub.publish(String(data=json.dumps(s)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('course')
    ap.add_argument('--setpoint-rate', type=float, default=10.0)
    ap.add_argument('--exit-when-done', action='store_true')
    a, rest = ap.parse_known_args()
    rclpy.init(args=rest)
    sp = SimParams(setpoint_rate_hz=a.setpoint_rate)
    node = KinSimRos(load_course(a.course), sp, a.exit_when_done)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

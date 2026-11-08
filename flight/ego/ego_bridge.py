#!/usr/bin/env python3
"""EGO-Planner bridge (runs in the skybeta-ego container). Never commands PX4: the dashboard decides.

  /claude/usb/out/vehicle_local_position + vehicle_attitude (PX4 NED / FRD)
      -> /ego/odom      nav_msgs/Odometry, ENU / FLU, frame 'world' (the dashboard's PX4-local ENU frame)
  /position_cmd (quadrotor_msgs/PositionCommand from EGO's traj_server)
      -> /ego/setpoint  std_msgs/String JSON {p, v, a, yaw, yaw_dot, id, stamp}
  -> /ego/status        std_msgs/String JSON, 2 Hz: odom/cmd ages + EGO's current plan (B-spline control points)
"""
import json, math, time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from quadrotor_msgs.msg import PositionCommand
from traj_utils.msg import Bspline

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
S = math.sqrt(.5)


def qmul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return (aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw)


def enu_flu(q_ned_frd):
    """PX4 attitude (FRD body -> NED, w x y z) -> FLU body -> ENU."""
    return qmul(qmul((0., S, S, 0.), tuple(q_ned_frd)), (0., 1., 0., 0.))


class Bridge(Node):
    def __init__(self):
        super().__init__('ego_bridge')
        self.att, self.t_odom, self.t_cmd = None, None, None
        self.path, self.t_path = None, None
        self.pub_odom = self.create_publisher(Odometry, '/ego/odom', 10)
        self.pub_sp = self.create_publisher(String, '/ego/setpoint', 10)
        self.pub_st = self.create_publisher(String, '/ego/status', 10)
        self.create_subscription(VehicleAttitude, '/claude/usb/out/vehicle_attitude', self._att, PX4_QOS)
        self.create_subscription(VehicleLocalPosition, '/claude/usb/out/vehicle_local_position', self._lpos, PX4_QOS)
        self.create_subscription(PositionCommand, '/position_cmd', self._cmd, 10)
        self.create_subscription(Bspline, '/planning/bspline', self._plan, 10)
        self.create_timer(.5, self._status)

    def _att(self, m):
        self.att = enu_flu(m.q)

    def _lpos(self, m):
        if self.att is None or not (m.xy_valid and m.z_valid):
            return
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = 'world', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = float(m.y), float(m.x), -float(m.z)
        w, x, y, z = self.att
        o.pose.pose.orientation.w, o.pose.pose.orientation.x = w, x
        o.pose.pose.orientation.y, o.pose.pose.orientation.z = y, z
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = float(m.vy), float(m.vx), -float(m.vz)
        self.pub_odom.publish(o)
        self.t_odom = time.monotonic()

    def _cmd(self, m):
        self.t_cmd = time.monotonic()
        self.pub_sp.publish(String(data=json.dumps(dict(
            p=[m.position.x, m.position.y, m.position.z], v=[m.velocity.x, m.velocity.y, m.velocity.z],
            a=[m.acceleration.x, m.acceleration.y, m.acceleration.z], yaw=m.yaw, yaw_dot=m.yaw_dot,
            id=int(m.trajectory_id), flag=int(m.trajectory_flag),
            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9))))

    def _plan(self, m):
        # ponytail: control points, not the sampled curve; the B-spline hugs them closely enough to draw
        pts = [[round(q.x, 2), round(q.y, 2), round(q.z, 2)] for q in m.pos_pts]
        self.path, self.t_path = pts[::max(1, len(pts)//40)], time.monotonic()

    def _status(self):
        now = time.monotonic()
        age = lambda t: None if t is None else round(now-t, 2)
        self.pub_st.publish(String(data=json.dumps(dict(odom_age_s=age(self.t_odom), cmd_age_s=age(self.t_cmd),
                                                     path=self.path, path_age_s=age(self.t_path)))))


def main():
    rclpy.init()
    rclpy.spin(Bridge())


if __name__ == '__main__':
    main()

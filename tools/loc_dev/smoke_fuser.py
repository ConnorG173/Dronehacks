#!/usr/bin/env python3
"""End-to-end smoke test of loc_fuser with fake cuVSLAM and PX4 inputs (no sim, no drone).

Run inside the dev container after `build_test.sh` (it starts loc_fuser itself):

    python3 /repo/tools/loc_dev/smoke_fuser.py --config /repo/planner/config/course_v3.yaml

Scenario (all timing on the wall clock):
  0-2 s    on the pad, disarmed then armed at 1 s. The stack "started elsewhere": odom pose of the
           pad is (1.0, 2.0, 0) with yaw 0.3 rad, so the anchor must not assume odom origin = pad.
  2-4 s    takeoff (landed false) and 2 m forward along the body x axis, 0.5 m up.
  4-5 s    vo_state = 2 (lost)          -> status ERROR
  5-5.6 s  VIO stops                    -> status ERROR (stale)
Checks: course pose matches the launch pose + body motion, header stamps copied, TF course -> map
present, status levels, and the receive latency of /team/odom_map behind our own VIO publish.
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from px4_msgs.msg import VehicleAttitude, VehicleLandDetected, VehicleStatus


def read_start(path):
    start = heading = None
    for line in open(path):
        line = line.split("#")[0].strip()
        if line.startswith("StartPos:"):
            start = [float(v) for v in line.split(":", 1)[1].replace("[", " ").replace("]", " ").replace(",", " ").split()]
        if line.startswith("StartHeadingDeg:"):
            heading = float(line.split(":", 1)[1])
    return np.array(start), math.radians(heading)


def yaw_q(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)  # x, y, z, w


class Smoke(Node):
    def __init__(self, start, heading, base_h):
        super().__init__("smoke_fuser")
        self.start, self.heading, self.base_h = start, heading, base_h
        self.pub_odom = self.create_publisher(Odometry, "/visual_slam/tracking/odometry", 10)
        self.pub_vo = self.create_publisher(VisualSlamStatus, "/visual_slam/status", 10)
        self.pub_att = self.create_publisher(VehicleAttitude, "/fmu/out/vehicle_attitude", qos_profile_sensor_data)
        self.pub_land = self.create_publisher(VehicleLandDetected, "/fmu/out/vehicle_land_detected", qos_profile_sensor_data)
        self.pub_vs = self.create_publisher(VehicleStatus, "/fmu/out/vehicle_status_v1", qos_profile_sensor_data)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)
        self.create_subscription(Odometry, "/team/odom_map", self.on_out, 50)
        self.create_subscription(DiagnosticStatus, "/team/loc/status", self.on_status, 50)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        self.sent = {}       # stamp ns -> (send wall time, expected course xyz)
        self.lat_ms, self.err_m, self.status, self.course_tf = [], [], [], 0
        self.t0 = time.monotonic()
        self.odom_origin = np.array([1.0, 2.0, 0.0])
        self.odom_yaw = 0.3
        self.create_timer(1 / 30, self.tick)

    def phase_t(self):
        return time.monotonic() - self.t0

    def tick(self):
        t = self.phase_t()
        now = self.get_clock().now().to_msg()
        armed = t > 1.0
        landed = t < 2.0
        s = min(max(t - 2.0, 0.0) / 2.0, 1.0)  # 0..1 over 2-4 s
        body = np.array([2.0 * s, 0.0, 0.5 * s])  # motion in the pad's body frame
        c, si = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        p_odom = self.odom_origin + np.array([c * body[0] - si * body[1], si * body[0] + c * body[1], body[2]])
        ch, sh = math.cos(self.heading), math.sin(self.heading)
        expect = self.start + np.array([0, 0, self.base_h]) + np.array(
            [ch * body[0] - sh * body[1], sh * body[0] + ch * body[1], body[2]])

        vs = VehicleStatus()
        vs.arming_state = VehicleStatus.ARMING_STATE_ARMED if armed else VehicleStatus.ARMING_STATE_DISARMED
        self.pub_vs.publish(vs)
        ld = VehicleLandDetected()
        ld.landed = landed
        self.pub_land.publish(ld)
        att = VehicleAttitude()
        # PX4 level; its heading is arbitrary (anchor uses roll/pitch only): NED yaw 1.0 rad.
        att.q = [math.cos(0.5), 0.0, 0.0, math.sin(0.5)]
        self.pub_att.publish(att)

        vo = VisualSlamStatus()
        vo.header.stamp = now
        vo.vo_state = 2 if 4.0 <= t < 5.0 else 1
        self.pub_vo.publish(vo)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.rotation.w = 1.0
        self.pub_tf.publish(TFMessage(transforms=[tf]))

        if t < 5.0:
            m = Odometry()
            m.header.stamp = now
            m.header.frame_id = "odom"
            m.child_frame_id = "base_link"
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = p_odom
            q = yaw_q(self.odom_yaw)
            m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
            m.twist.twist.linear.x = 1.0 if 2.0 <= t < 4.0 else 0.0
            self.sent[now.sec * 10**9 + now.nanosec] = (time.monotonic(), expect)
            self.pub_odom.publish(m)

    def on_out(self, m):
        k = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if k not in self.sent:
            return
        t_send, expect = self.sent[k]
        self.lat_ms.append((time.monotonic() - t_send) * 1000)
        p = m.pose.pose.position
        self.err_m.append((self.phase_t(), float(np.linalg.norm(np.array([p.x, p.y, p.z]) - expect)),
                           m.header.frame_id, m.twist.twist.linear.x))

    def on_status(self, s):
        level = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
        self.status.append((self.phase_t(), level, s.message, {kv.key: kv.value for kv in s.values}))

    def on_tf(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id == "course" and tr.child_frame_id == "map":
                self.course_tf += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--base-height", type=float, default=0.12)
    ap.add_argument("--no-launch", action="store_true", help="loc_fuser is already running")
    a = ap.parse_args()
    start, heading = read_start(a.config)

    proc = None
    if not a.no_launch:
        proc = subprocess.Popen(["ros2", "run", "team_loc", "loc_fuser", "--ros-args",
                                 "-p", f"course_config:={a.config}", "-p", f"base_height_on_pad_m:={a.base_height}"],
                                start_new_session=True)  # own process group: ros2 run forks the node
        time.sleep(1.5)
    rclpy.init()
    n = Smoke(start, heading, a.base_height)
    end = time.monotonic() + 6.0
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.01)
    if proc:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)

    ok = True

    def check(cond, what):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + what)
        ok &= bool(cond)

    errs = [e for e in n.err_m if e[0] > 0.3]  # skip the first anchor instant
    check(len(errs) > 100, f"odom_map messages received: {len(n.err_m)}")
    if errs:
        mx = max(e[1] for e in errs)
        check(mx < 1e-3, f"course pose matches launch pose + body motion (max err {mx*1000:.3f} mm)")
        check(all(e[2] == "course" for e in errs), "frame_id is 'course'")
        check(any(abs(e[3] - 1.0) < 1e-9 for e in errs), "twist copied through")
    lat = np.array(n.lat_ms)
    if len(lat):
        print(f"latency ms (our publish -> odom_map received): p50 {np.percentile(lat,50):.2f}  "
              f"p95 {np.percentile(lat,95):.2f}  max {lat.max():.2f}")
        check(np.percentile(lat, 95) < 5.0, "latency p95 < 5 ms (includes two DDS hops)")
    check(n.course_tf > 50, f"TF course -> map published ({n.course_tf})")

    def levels(t0, t1):
        return {s[1] for s in n.status if t0 <= s[0] < t1}
    check(levels(2.3, 3.9) == {0}, f"status OK while flying: {levels(2.3, 3.9)}")
    check(levels(4.2, 4.9) == {2}, f"status ERROR while vo_state=2: {levels(4.2, 4.9)}")
    check(levels(5.4, 5.9) == {2}, f"status ERROR when VIO stops: {levels(5.4, 5.9)}")
    tilt = {s[3].get("anchor_tilt_source") for s in n.status if 2.3 <= s[0] < 3.9}
    check(tilt == {"px4_attitude"}, f"anchor tilt from PX4 attitude at takeoff: {tilt}")
    anchor = {s[3].get("anchor") for s in n.status if 2.3 <= s[0] < 3.9}
    check(anchor == {"frozen"}, f"anchor frozen in flight: {anchor}")
    msgs = {s[2] for s in n.status if 5.4 <= s[0] < 5.9}
    check(any("stale" in m for m in msgs), f"stale reason reported: {msgs}")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

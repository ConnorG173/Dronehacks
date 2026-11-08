#!/usr/bin/env python3
"""IMU feed for the SHADOW cuVSLAM (team_imu namespace). Publishes /team_imu/imu only; reads nothing from
PX4 except, in px4 mode, /fmu/out/sensor_combined.

  oak (default): /oak/imu/data (BNO086 via depthai, 200 Hz, same device clock as the stereo images)
                 -> accel x calib.accel_scale, frame_id team_imu_frame. Axes untouched: the measured
                 IMU->base_link rotation is carried by the static TF base_link -> team_imu_frame
                 (shadow.sh). The depthai TF for oak_imu_frame is ~92 deg wrong (imu_calib.json), which
                 is why we never use it.
  px4:           /fmu/out/sensor_combined (FRD, board rotation applied, ~77 Hz over uXRCE-DDS, stamped on
                 the Jetson clock by uXRCE time sync) -> FLU, frame team_imu_frame (= base_link axes).

Both: header stamps are kept (cuVSLAM integrates on them); a sample whose stamp does not increase is
dropped, never re-stamped. Counts drops and prints rate/gap stats every 10 s.
"""
import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu

HERE = os.path.dirname(os.path.abspath(__file__))
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,  # matches any publisher
                     history=HistoryPolicy.KEEP_LAST, depth=1)
FRAME = 'team_imu_frame'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('source', nargs='?', default='oak', choices=['oak', 'px4'])
    ap.add_argument('--calib', default=os.path.join(HERE, 'calib.json'))
    ap.add_argument('--px4-topic', default='/fmu/out/sensor_combined',
                    help='px4 mode: /fmu/out/sensor_combined (UART DDS) or /claude/usb/out/sensor_combined (USB adapter)')
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = Node('team_imu_prep')
    pub = n.create_publisher(Imu, '/team_imu/imu', qos_profile_sensor_data)
    st = {'last': -1.0, 'n': 0, 'drop': 0, 'maxgap': 0.0, 't0': time.time(), 'age': []}

    def emit(m, t):
        if t <= st['last']:
            st['drop'] += 1
            return
        if st['last'] > 0:
            st['maxgap'] = max(st['maxgap'], t - st['last'])
        st['last'] = t
        st['n'] += 1
        if len(st['age']) < 2000:
            st['age'].append(time.time() - t)        # arrival minus stamp: clock offset + latency
        pub.publish(m)

    if a.source == 'oak':
        k = float(json.load(open(a.calib)).get('accel_scale', 1.0))

        def on_oak(m):
            m.header.frame_id = FRAME
            m.linear_acceleration.x *= k
            m.linear_acceleration.y *= k
            m.linear_acceleration.z *= k
            emit(m, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

        n.create_subscription(Imu, '/oak/imu/data', on_oak, qos_profile_sensor_data)
        n.get_logger().info(f'/oak/imu/data -> /team_imu/imu (accel x{k:.4f}, frame {FRAME})')
    else:
        from px4_msgs.msg import SensorCombined

        def on_px4(s):
            t_us = int(s.timestamp)
            o = Imu()
            o.header.stamp.sec, o.header.stamp.nanosec = t_us // 1_000_000, (t_us % 1_000_000) * 1000
            o.header.frame_id = FRAME
            g, acc = s.gyro_rad, s.accelerometer_m_s2
            o.angular_velocity.x, o.angular_velocity.y, o.angular_velocity.z = float(g[0]), -float(g[1]), -float(g[2])
            o.linear_acceleration.x, o.linear_acceleration.y, o.linear_acceleration.z = (
                float(acc[0]), -float(acc[1]), -float(acc[2]))
            o.orientation_covariance[0] = -1.0          # no orientation
            emit(o, t_us * 1e-6)

        n.create_subscription(SensorCombined, a.px4_topic, on_px4, PX4_QOS)
        n.get_logger().info(f'{a.px4_topic} (FRD) -> /team_imu/imu (FLU, frame {FRAME})')

    def report():
        dt = time.time() - st['t0']
        n.get_logger().info(f"{st['n'] / max(dt, 1e-9):.1f} Hz out, {st['drop']} non-increasing stamps dropped, "
                            f"max stamp gap {st['maxgap'] * 1000:.1f} ms, arrival-stamp median "
                            f"{(sorted(st['age'])[len(st['age']) // 2] * 1000 if st['age'] else float('nan')):.1f} ms")
        st.update(n=0, drop=0, maxgap=0.0, t0=time.time(), age=[])

    n.create_timer(10.0, report)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

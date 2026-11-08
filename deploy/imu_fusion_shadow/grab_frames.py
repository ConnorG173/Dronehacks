#!/usr/bin/env python3
"""Read-only: save /oak/left/image_mono as JPEG at --hz (default 2) with its header stamp in the file name,
so VIO jumps in a log can be matched to what the camera saw. Publishes nothing.
    python3 grab_frames.py --out DIR [--hz 2] [--scale 0.5]"""
import argparse
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--hz', type=float, default=2.0)
    ap.add_argument('--scale', type=float, default=0.5)
    a = ap.parse_args()
    import cv2
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    n = Node('team_imu_grab')
    last = [0.0]

    def cb(m):
        now = time.time()
        if now - last[0] < 1.0 / a.hz:
            return
        last[0] = now
        img = np.frombuffer(bytes(m.data), np.uint8)[: m.height * m.step].reshape(m.height, m.step)[:, : m.width]
        if a.scale != 1.0:
            img = cv2.resize(img, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_AREA)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        cv2.imwrite(os.path.join(a.out, f'{stamp:.3f}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 70])

    n.create_subscription(Image, '/oak/left/image_mono', cb, qos_profile_sensor_data)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Pixel statistics of a few frames from camera topics (blank / frozen camera check).
    python3 image_stats.py /oak/left/image_mono /oak/right/image_mono"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

topics = sys.argv[1:] or ['/oak/left/image_mono']
rclpy.init()
n = Node('pf_image_stats')
frames = {t: [] for t in topics}


def cb(topic):
    def f(m):
        if len(frames[topic]) < 3:
            a = np.frombuffer(bytes(m.data), np.uint8)
            frames[topic].append((m.encoding, m.width, m.height, a))
    return f


for t in topics:
    n.create_subscription(Image, t, cb(t), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 8 and any(len(v) < 3 for v in frames.values()):
    rclpy.spin_once(n, timeout_sec=0.2)
for t, fs in frames.items():
    for k, (enc, w, h, a) in enumerate(fs):
        diff = '' if k == 0 else f', mean |diff| vs first {np.mean(np.abs(a.astype(int) - fs[0][3].astype(int))):.2f}'
        print(f'{t} [{k}] {enc} {w}x{h}: mean {a.mean():.1f} std {a.std():.1f} min {a.min()} max {a.max()}{diff}')
    if not fs:
        print(f'{t}: no frames in 8 s')

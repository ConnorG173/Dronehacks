#!/usr/bin/env python3
"""Wait on the VM host for /team/mission/state to reach done/abort (or a timeout).
Prints state changes with ground-truth position. Exit 0 on done, 1 otherwise.
    python3 wait_mission.py [timeout_s]"""
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
rclpy.init()
n = Node('pf_wait_mission')
st = {'state': None, 'p': None}


def on_state(m):
    if m.data != st['state']:
        p = st['p']
        where = f' gt ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})' if p is not None else ''
        print(f'{time.strftime("%H:%M:%S")} mission: {m.data}{where}', flush=True)
    st['state'] = m.data


n.create_subscription(String, '/team/mission/state', on_state, 10)
n.create_subscription(Odometry, '/sim/ground_truth/odom', lambda m: st.update(p=m.pose.pose.position), 10)
t0 = time.time()
while time.time() - t0 < timeout and st['state'] not in ('done', 'abort'):
    rclpy.spin_once(n, timeout_sec=0.2)
print(f'final: {st["state"]} after {time.time() - t0:.1f} s', flush=True)
sys.exit(0 if st['state'] == 'done' else 1)

#!/usr/bin/env python3
"""Guard for the unattended cuVSLAM build (build_vslam.sh). Read-only towards PX4; publishes nothing.

    python3 build_guard.py --pgid <build process group> --status <file>

Every 0.5 s: if PX4 reports ARMED, NOT LANDED, or its telemetry (/claude/usb/out/vehicle_status_v1,
vehicle_land_detected) is older than 3 s, the whole build process group is frozen at once (SIGSTOP).
It is resumed (SIGCONT) only after 10 s of fresh disarmed + landed telemetry. Exits when the build is gone.
The status file holds one word plus detail: RUNNING, PAUSED (reason), then build_vslam.sh writes DONE/FAILED.
"""
import argparse
import os
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)


def alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pgid', type=int, required=True)
    ap.add_argument('--status', required=True)
    a = ap.parse_args()
    from px4_msgs.msg import VehicleLandDetected, VehicleStatus
    rclpy.init()
    n = Node('team_imu_build_guard')
    s = {'armed': None, 'landed': None, 'ts': 0.0, 'tl': 0.0}
    n.create_subscription(VehicleStatus, '/claude/usb/out/vehicle_status_v1',
                          lambda m: s.update(armed=int(m.arming_state) == 2, ts=time.time()), QOS)
    n.create_subscription(VehicleLandDetected, '/claude/usb/out/vehicle_land_detected',
                          lambda m: s.update(landed=bool(m.landed), tl=time.time()), QOS)
    paused, safe_since, last = False, None, 0.0

    def status(word):
        with open(a.status, 'w') as f:
            f.write(f'{word} {time.strftime("%F %T")}\n')
        print(f'{time.strftime("%T")} {word}', flush=True)

    status('RUNNING')
    while alive(a.pgid):
        t_end = time.time() + 0.5
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.05)
        now = time.time()
        fresh = now - s['ts'] < 3.0 and now - s['tl'] < 3.0
        safe = fresh and s['armed'] is False and s['landed'] is True
        if not safe:
            safe_since = None
            if not paused:
                os.killpg(a.pgid, signal.SIGSTOP)
                paused = True
                status(f"PAUSED armed={s['armed']} landed={s['landed']} telemetry_fresh={fresh}")
        else:
            safe_since = safe_since or now
            if paused and now - safe_since >= 10.0:
                os.killpg(a.pgid, signal.SIGCONT)
                paused = False
                status('RUNNING (resumed: disarmed + landed for 10 s)')
        if now - last > 60:
            last = now
            print(f'{time.strftime("%T")} alive, paused={paused}', flush=True)
    if paused:
        try:
            os.killpg(a.pgid, signal.SIGCONT)
        except ProcessLookupError:
            pass


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Defer all overlay imports until fresh USB telemetry confirms the ground.

Uses only the standard library. In particular, waiting here must not import ROS,
OpenCV, NumPy, Torch, TensorRT, or the worker. It sends no vehicle commands.
"""
import json
import math
import os
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parent
STATUS = ROOT.parent / 'mavlink_usb' / 'runtime-status.json'
MAX_STATUS_BYTES = 65536


def nonnegative_number(value):
    try:
        return (type(value) in (int, float) and math.isfinite(value) and value >= 0)
    except OverflowError:
        return False


def blocked_reason(status, now_ns):
    """None means grounded; ages include time since the adapter wrote the file."""
    if not isinstance(status, dict):
        return 'USB status is not an object'
    stamp = status.get('wall_ns')
    if type(stamp) is not int or stamp <= 0:
        return 'USB status has no valid timestamp'
    delta_ns = now_ns - stamp
    if not 0 <= delta_ns <= 2_000_000_000:
        return 'USB status is stale or from the future'
    file_age = delta_ns / 1e9
    if status.get('router_alive') is not True or status.get('connected') is not True:
        return 'USB connection is unavailable'
    if status.get('fc_armed') is not False or status.get('fc_landed') is not True:
        return 'PX4 is armed, airborne, or its ground state is unknown'
    heartbeat_age = status.get('fc_heartbeat_age_s')
    land_age = status.get('fc_landed_age_s')
    # The adapter saves once per second. Include that file age instead of
    # treating a cached heartbeat/landed sample as newly received on each poll.
    if (not nonnegative_number(heartbeat_age) or heartbeat_age > 1.5
            or heartbeat_age + file_age > 2.5):
        return 'PX4 heartbeat is stale or missing'
    if (not nonnegative_number(land_age) or land_age > 1.0
            or land_age + file_age > 1.5):
        return 'PX4 landed state is stale or missing'
    return None


def read_ground_state(path, now_ns):
    try:
        with Path(path).open('r', encoding='utf-8') as handle:
            data = handle.read(MAX_STATUS_BYTES + 1)
        if len(data) > MAX_STATUS_BYTES:
            return 'USB status is oversized'
        status = json.loads(data)
    except (OSError, ValueError, UnicodeError):
        return 'USB status is unavailable or incomplete'
    return blocked_reason(status, now_ns)


class GroundedGate:
    def __init__(self, stable_s=2.0):
        self.stable_s = stable_s
        self.since = None

    def update(self, now, reason):
        if reason is not None:
            self.since = None
            return False
        if self.since is None or now < self.since:
            self.since = now
        return now - self.since >= self.stable_s


def worker_path(root=ROOT):
    root = Path(root).resolve()
    worker = (root / 'worker.py').resolve(strict=True)
    if worker.parent != root or not worker.is_file():
        raise RuntimeError('Overlay worker must be a file in the guard directory')
    return worker


def main(args=None):
    args = sys.argv[1:] if args is None else args
    worker = worker_path()
    gate = GroundedGate()
    previous = object()
    while True:
        reason = read_ground_state(STATUS, time.time_ns())
        message = reason or 'waiting for two seconds of fresh disarmed/landed telemetry'
        if message != previous:
            print('Depth overlay: ' + message, flush=True)
            previous = message
        if gate.update(time.monotonic(), reason):
            # Recheck immediately before exec. The worker also guards model
            # loading with its own fresh ROS disarmed/landed observations.
            reason = read_ground_state(STATUS, time.time_ns())
            if reason is None:
                print('Depth overlay: ground confirmed; starting worker', flush=True)
                os.execv(sys.executable, [sys.executable, '-u', str(worker), *args])
            gate.update(time.monotonic(), reason)
        time.sleep(.1)


if __name__ == '__main__':
    main()

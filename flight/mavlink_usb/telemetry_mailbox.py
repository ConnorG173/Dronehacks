"""Bound return telemetry without replaying cached samples as fresh data."""
from collections import deque
import threading
import time


TYPES = frozenset(('HEARTBEAT', 'EXTENDED_SYS_STATE', 'ODOMETRY',
                  'ATTITUDE_QUATERNION', 'HIGHRES_IMU', 'SYS_STATUS',
                  'COMMAND_ACK'))


class TelemetryMailbox:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}
        self.acks = deque(maxlen=32)
        self.received = {}
        self.stamps = {}

    def put(self, message, now=None, wall_us=None):
        kind = message.get_type()
        if kind not in TYPES:
            return False
        now = time.monotonic() if now is None else now
        wall_us = time.time_ns() // 1000 if wall_us is None else wall_us
        stamp = getattr(message, 'time_usec', getattr(message, 'time_boot_ms', None))
        with self.lock:
            # Timestamped streams must actually advance. A repeated frozen
            # position must not keep the dashboard's freshness check alive.
            if stamp is not None and stamp <= self.stamps.get(kind, -1):
                return False
            if stamp is not None:
                self.stamps[kind] = stamp
            self.received[kind] = now
            item = (message, now, wall_us)
            if kind == 'COMMAND_ACK':
                self.acks.append(item)
            else:
                self.latest[kind] = item
        return True

    def take(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            values = list(self.latest.values()) + list(self.acks)
            self.latest.clear()
            self.acks.clear()
        return [item for item in values if 0 <= now - item[1] <= .25]

    def ages(self):
        now = time.monotonic()
        with self.lock:
            return {kind: round(now - at, 3) for kind, at in self.received.items()}

    def clear(self):
        with self.lock:
            self.latest.clear()
            self.acks.clear()
            self.received.clear()
            self.stamps.clear()

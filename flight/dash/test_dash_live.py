#!/usr/bin/env python3
"""On the drone, inside skybeta-stack, against the running dash_node (DRY-RUN): heartbeat dead-man test.
Counts messages on the setpoint topic while heartbeating, then after the heartbeat stops."""
if __name__ != '__main__':
    import unittest
    raise unittest.SkipTest('Manual dry-run probe; never execute during test discovery')

import json, threading, time, urllib.request
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped

BASE = "http://100.64.16.93:8091"
TOK = open("/root/team_ws/claude/dash/dash_token.txt").read().strip()
CL = "testclient01"


def get(path):
    with urllib.request.urlopen(f"{BASE}{path}{'&' if '?' in path else '?'}k={TOK}", timeout=2) as r:
        return json.loads(r.read())


def post(op, **kw):
    req = urllib.request.Request(f"{BASE}/api/cmd?k={TOK}", data=json.dumps(dict(c=CL, op=op, **kw)).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=2) as r:
        j = json.loads(r.read())
    print(f"  {op}: {j['ok']} {j['msg']}")
    return j


if get('/api/state').get('live', True) is not False:
    raise SystemExit('Refusing to run the heartbeat test against a LIVE dashboard')

rclpy.init()
n = Node("claude_dash_test")
stamps = []
n.create_subscription(PoseStamped, "/claude/dash/setpoint_preview/pose", lambda m: stamps.append(time.monotonic()), 10)
n.create_subscription(TwistStamped, "/claude/dash/setpoint_preview/twist", lambda m: stamps.append(time.monotonic()), 10)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()

try:
    urllib.request.urlopen(f"{BASE}/api/state?k=wrong", timeout=2)
    print("token check: FAILED (no 403)")
except urllib.error.HTTPError as e:
    print(f"token check: HTTP {e.code} without the right token")
s = get(f"/api/state?c={CL}")
print(f"start: mode={s['mode']} live={s['live']} pos_valid={s['pos'] and s['pos']['valid']} px4_link={s['fc'] and s['fc']['px4_link']}")
post("claim")
post("engage", alt=1.0)
t0 = time.monotonic()
while time.monotonic() - t0 < 3.0:                      # heartbeat at 10 Hz
    get(f"/api/state?c={CL}")
    time.sleep(0.1)
n_hb = len([t for t in stamps if t >= t0])
s = get(f"/api/state?c={CL}")
print(f"heartbeating 3 s: {n_hb} setpoints ({n_hb / 3:.1f} Hz), mode={s['mode']}, out={s['out']}")
t_quiet = time.monotonic()
time.sleep(2.0)                                         # silence (no polls from CL)
after = [t for t in stamps if t > t_quiet]
last = max(after) if after else t_quiet
s = get("/api/state?c=observer")                        # a non-owner poll is not a heartbeat
print(f"silent 2 s: setpoints stopped {last - t_quiet:.2f} s after the last heartbeat; mode={s['mode']} reason='{s['reason']}'")
cnt = len(stamps)
for _ in range(10):
    get(f"/api/state?c={CL}")
    time.sleep(0.1)
print(f"heartbeat back 1 s: {len(stamps) - cnt} setpoints (must be 0: failsafe is latched), mode={get(f'/api/state?c={CL}')['mode']}")
post("engage", alt=1.0)                                 # refused until reset
post("reset")
post("release")
print("events:", [e[1] for e in get(f"/api/state?c={CL}")["events"]][-4:])
rclpy.shutdown()

#!/usr/bin/env python3
"""One-line drone health (read-only), inside skybeta-stack: PX4 link, positions, height frames, EKF resets, tilt."""
import json, os, time, urllib.request
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

Q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               history=HistoryPolicy.KEEP_LAST, depth=1)
rclpy.init()
n = Node("claude_health")
st, lp = [None], []
n.create_subscription(String, "/skybeta/state", lambda m: st.__setitem__(0, json.loads(m.data)), 10)
from px4_msgs.msg import VehicleLocalPosition
n.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                      lambda m: lp.append((time.time(), int(m.xy_reset_counter), int(m.heading_reset_counter))), Q)
t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
s = st[0] or {}
f = lambda v: "-" if not v else "(" + ",".join(f"{x:+.2f}" for x in v) + ")"
if len(lp) >= 2:
    dt = lp[-1][0] - lp[0][0]
    rs = f"xy resets +{lp[-1][1] - lp[0][1]} / heading +{lp[-1][2] - lp[0][2]} in {dt:.0f} s"
else:
    rs = "no PX4 local position"
o, l = s.get("origin_enu"), s.get("local_enu")
fo = None if not (o and l) else o[2] - l[2]
try:
    lv = json.load(open("/root/team_ws/claude/vio_level_status.json"))
    tilt = f"tilt cuVSLAM {lv.get('tilt_raw')} vs PX4 {lv.get('tilt_px4')} deg"
except Exception:
    tilt = "tilt: vio_level not reporting"
print(f"{time.strftime('%T')} px4_link={s.get('px4_link')} landed={s.get('landed')} | odom {f(s.get('odom_enu'))} "
      f"local {f(l)} | height frames differ {'-' if fo is None else f'{fo:+.2f}'} m | {rs} | {tilt}", flush=True)
rclpy.shutdown()

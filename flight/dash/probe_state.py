# read-only: sample /skybeta/state for N seconds, print odom vs PX4 local positions
import json, sys, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init(); n = Node("claude_probe_state"); last = [None]
n.create_subscription(String, "/skybeta/state", lambda m: last.__setitem__(0, json.loads(m.data)), 10)
t0 = time.time(); tp = 0
while time.time() - t0 < float(sys.argv[1] if len(sys.argv) > 1 else 6):
    rclpy.spin_once(n, timeout_sec=0.1)
    if last[0] and time.time() - tp > 1.0:
        s = last[0]; tp = time.time()
        f = lambda v: "(" + ", ".join(f"{x:+.2f}" for x in v) + ")" if v else "-"
        print(f"{time.strftime('%T')} odom {f(s.get('odom_enu'))}  px4 local {f(s.get('local_enu'))}  origin {f(s.get('origin_enu'))}  landed={s.get('landed')} nav={s.get('nav_state_name')}", flush=True)
rclpy.shutdown()

# usage: hz.py SECONDS  -> rate and p95 gap of camera, VIO odometry
import sys, time, rclpy, numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
T = float(sys.argv[1]) if len(sys.argv) > 1 else 10
rclpy.init(); n = rclpy.create_node("hz_probe"); st = {}
def cb(name):
    st[name] = []
    return lambda m: st[name].append(time.monotonic())
for t, ty in [("/oak/left/image_raw", Image), ("/oak/right/image_raw", Image), ("/visual_slam/tracking/odometry", Odometry)]:
    n.create_subscription(ty, t, cb(t), qos_profile_sensor_data)
end = time.monotonic() + T
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=.1)
for k, v in st.items():
    if len(v) < 3: print(f"{k}: {len(v)} msgs"); continue
    g = np.diff(v) * 1000
    print(f"{k}: {(len(v)-1)/(v[-1]-v[0]):.1f} Hz  p95 gap {np.percentile(g,95):.1f} ms  max {g.max():.0f} ms")

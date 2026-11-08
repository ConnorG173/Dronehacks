# usage: yaw_probe.py SECONDS -> yaw of raw and levelled VIO, largest per-message yaw step and position step
import sys, time, math, rclpy
from nav_msgs.msg import Odometry
T = float(sys.argv[1]) if len(sys.argv) > 1 else 15
rclpy.init(); n = rclpy.create_node("yaw_probe"); st = {}
def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
def cb(name):
    st[name] = dict(n=0, first=None, last=None, max_dyaw=0., max_dp=0., prev=None)
    def f(m):
        s = st[name]; y = yaw(m.pose.pose.orientation); p = m.pose.pose.position
        if s["prev"] is not None:
            dy = abs((y - s["prev"][0] + 180) % 360 - 180); dp = math.dist((p.x,p.y,p.z), s["prev"][1])
            s["max_dyaw"] = max(s["max_dyaw"], dy); s["max_dp"] = max(s["max_dp"], dp)
        s["prev"] = (y, (p.x,p.y,p.z)); s["n"] += 1; s["first"] = s["first"] or (y,(p.x,p.y,p.z)); s["last"] = (y,(p.x,p.y,p.z))
    return f
for t in ["/visual_slam/tracking/odometry", "/claude/vio_leveled/odometry", "/team_heading/odometry"]:
    n.create_subscription(Odometry, t, cb(t), 10)
end = time.monotonic() + T
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=.1)
for k, s in st.items():
    if not s["n"]: print(k, "no msgs"); continue
    print(f"{k}: n={s[\"n\"]} yaw {s[\"first\"][0]:.1f} -> {s[\"last\"][0]:.1f}  max step yaw {s[\"max_dyaw\"]:.2f} deg  pos {s[\"max_dp\"]*100:.1f} cm  pos now ({s[\"last\"][1][0]:.2f},{s[\"last\"][1][1]:.2f},{s[\"last\"][1][2]:.2f})")

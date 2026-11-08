#!/usr/bin/env bash
# Does cuVSLAM's odometry follow the sim drone? Fly the example and print VIO vs ground truth.
#   SIM_ID=1 bash vio_vs_gt.sh
source "$(dirname "$0")/sim/scripts/common.sh"
X="docker exec -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/team/sim/dds/fastdds_udp.xml $CONTAINER bash -lc"
cat > /tmp/vio_cmp_$SIM_ID.py <<'EOF'
import sys, time, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
dur = float(sys.argv[1]); rclpy.init(); n = Node("vio_cmp"); s = {"vo": 0, "img": 0}
n.create_subscription(Odometry, "/visual_slam/tracking/odometry", lambda m: s.update(vio=m, vo=s["vo"] + 1), 10)
n.create_subscription(Odometry, "/sim/ground_truth/odom", lambda m: s.update(gt=m), 10)
n.create_subscription(VisualSlamStatus, "/visual_slam/status", lambda m: s.update(st=m), 10)
t0 = time.time(); nxt = 0
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - t0 >= nxt and "gt" in s:
        v = s.get("vio"); g = s["gt"].pose.pose.position
        vs = f"vio=({v.pose.pose.position.x:6.3f},{v.pose.pose.position.y:6.3f},{v.pose.pose.position.z:6.3f}) stamp={v.header.stamp.sec}.{v.header.stamp.nanosec//1000000:03d}" if v else "vio=none"
        st = s.get("st")
        print(f"t={time.time()-t0:5.1f} {vs} | gt=({g.x:6.2f},{g.y:6.2f},{g.z:5.2f}) | vo_state={getattr(st,'vo_state','?')} msgs={s['vo']}", flush=True)
        nxt += 3
EOF
docker cp /tmp/vio_cmp_$SIM_ID.py "$CONTAINER":/tmp/vio_cmp.py >/dev/null
tmux kill-window -t "$TMUX_SESSION:competitor" 2>/dev/null || true
tmux new-window -t "$TMUX_SESSION" -n competitor "SIM_ID=$SIM_ID bash $REPO/sim/scripts/competitor.sh; bash"
sleep 12
timeout 260 python3 "$REPO/sim/pilot.py" --sim "$SIM_ID" go | grep -E "mode is|refused|never"
$X "source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && python3 /tmp/vio_cmp.py 24"
timeout 30 python3 "$REPO/sim/pilot.py" --sim "$SIM_ID" land >/dev/null
echo "--- image timestamps advancing?"
$X "source /opt/ros/humble/setup.bash && for i in 1 2 3; do timeout 3 ros2 topic echo --once --field header.stamp /oak/left/image_mono 2>/dev/null | tr '\n' ' '; echo; sleep 1; done"

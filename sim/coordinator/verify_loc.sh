#!/usr/bin/env bash
# Coordinator check of localization M1 (loc_fuser) in a sim slot: build, run, fly, compare with ground truth.
#   SIM_ID=1 bash verify_loc.sh
source "$(dirname "$0")/sim/scripts/common.sh"
CFG=/root/team/planner/config/course_v3.yaml
X="docker exec -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/team/sim/dds/fastdds_udp.xml $CONTAINER bash -lc"
echo "=== build team_loc"
$X "source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && cd /root/team && colcon build --packages-select team_loc --cmake-args -DBUILD_TESTING=OFF 2>&1 | tail -3"
echo "=== start loc_fuser"
tmux kill-window -t "$TMUX_SESSION:loc" 2>/dev/null || true
tmux new-window -t "$TMUX_SESSION" -n loc "$X 'source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && source /root/team/install/setup.bash && ros2 launch team_loc team_loc.launch.py course_config:=$CFG base_height_on_pad_m:=0.0'; bash"
sleep 10
cat > /tmp/loc_cmp_$SIM_ID.py <<'EOF'
import math, sys, time, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticStatus
dur = float(sys.argv[1]); rclpy.init(); n = Node("loc_cmp")
s = {}
n.create_subscription(Odometry, "/team/odom_map", lambda m: s.update(loc=m), 10)
n.create_subscription(Odometry, "/sim/ground_truth/odom", lambda m: s.update(gt=m), 10)
n.create_subscription(DiagnosticStatus, "/team/loc/status", lambda m: s.update(st=m), 10)
def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
t0 = time.time(); nxt = 0; errs = []
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
    if "loc" in s and "gt" in s:
        a, b = s["loc"].pose.pose, s["gt"].pose.pose
        e = math.dist((a.position.x, a.position.y), (b.position.x, b.position.y)); errs.append(e)
        if time.time() - t0 >= nxt:
            st = s.get("st"); lvl = st.level[0] if st and isinstance(st.level, (bytes, bytearray)) else (st.level if st else "?")
            print(f"t={time.time()-t0:5.1f} loc=({a.position.x:6.2f},{a.position.y:6.2f},{a.position.z:5.2f}) yaw={yaw(a.orientation):6.1f} | "
                  f"gt=({b.position.x:6.2f},{b.position.y:6.2f},{b.position.z:5.2f}) yaw={yaw(b.orientation):6.1f} | xy_err={e:.3f} "
                  f"frame={s['loc'].header.frame_id} status={lvl} {st.message if st else ''}", flush=True)
            nxt += 4
if errs:
    errs.sort(); print(f"xy error: median {errs[len(errs)//2]:.3f} m, p95 {errs[int(0.95*len(errs))]:.3f} m, max {errs[-1]:.3f} m over {len(errs)} samples")
else:
    print("no /team/odom_map or ground truth received")
EOF
docker cp /tmp/loc_cmp_$SIM_ID.py "$CONTAINER":/tmp/loc_cmp.py
echo "=== before takeoff"
$X "source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && python3 /tmp/loc_cmp.py 5"
echo "=== flying the organizers' example"
tmux kill-window -t "$TMUX_SESSION:competitor" 2>/dev/null || true
tmux new-window -t "$TMUX_SESSION" -n competitor "SIM_ID=$SIM_ID bash $REPO/sim/scripts/competitor.sh; bash"
sleep 12
timeout 260 python3 "$REPO/sim/pilot.py" --sim "$SIM_ID" go | grep -E "mode is|refused|never"
$X "source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && python3 /tmp/loc_cmp.py 38"
timeout 30 python3 "$REPO/sim/pilot.py" --sim "$SIM_ID" land >/dev/null
echo "=== loc_fuser log"
tmux capture-pane -p -J -t "$TMUX_SESSION:loc" -S -200 | grep -vE "^\s*$" | grep -iE "anchor|warn|error|status|frame" | tail -10

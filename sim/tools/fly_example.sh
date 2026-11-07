#!/usr/bin/env bash
# End-to-end smoke test for one slot (run on the VM):  SIM_ID=0 bash sim/tools/fly_example.sh [seconds]
# Starts the organizers' competitor example (flight interface + square) in tmux window "competitor",
# plays the safety pilot (arm, then Position = go), logs ground truth + flight-interface state, then lands.
source "$(dirname "$0")/../scripts/common.sh"
DUR="${1:-60}"
tmux kill-window -t "$TMUX_SESSION:competitor" 2>/dev/null || true
tmux new-window -t "$TMUX_SESSION" -n competitor "SIM_ID=$SIM_ID bash $REPO/sim/scripts/competitor.sh; bash"
sleep 12
echo "--- pilot go"
timeout 260 python3 "$REPO/sim/pilot.py" --sim "$SIM_ID" go
python3 - "$DUR" <<'EOF'
import sys, time, threading, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
dur = float(sys.argv[1])
rclpy.init(); n = Node("smoke_gt")
last = {}
n.create_subscription(Odometry, "/sim/ground_truth/odom", lambda m: last.update(p=m.pose.pose.position, v=m.twist.twist.linear), 10)
t0 = time.time(); nxt = 0
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time() - t0 >= nxt and "p" in last:
        p = last["p"]; print(f"t={time.time()-t0:5.1f}s  gt x={p.x:6.2f} y={p.y:6.2f} z={p.z:5.2f}", flush=True); nxt += 5
EOF
echo "--- flight interface (last lines)"
tmux capture-pane -p -J -t "$TMUX_SESSION:competitor" -S -200 | grep -iE "offboard|takeoff|square|clamp|error|warn" | tail -10
echo "--- pilot land"
timeout 30 python3 "$REPO/sim/pilot.py" --sim "$SIM_ID" land

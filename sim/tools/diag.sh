#!/usr/bin/env bash
# Sim diagnostics for one slot (run on the VM): processes, Gazebo clock/RTF, IMU message timing.
source "$(dirname "$0")/../scripts/common.sh"
echo "--- gz / px4 processes"
pgrep -af "gz sim|bin/px4|parameter_bridge" | cut -c1-150
echo "--- PX4 console: first errors"
tmux capture-pane -p -J -t "$TMUX_SESSION:px4" -S -3000 | grep -nE "ERROR|WARN" | grep -v "vehicle_imu" | head -15
tmux capture-pane -p -J -t "$TMUX_SESSION:px4" -S -3000 | grep -n "vehicle_imu" | head -2 | cut -c1-200
echo "--- gz world stats (real time factor)"
timeout 5 gz topic -e -t "/world/$WORLD/stats" -n 1 | grep -E "real_time_factor|sim_time|paused" | head -6
echo "--- gz IMU topic publishers"
gz topic -i -t "/world/$WORLD/model/$MODEL_NAME/link/base_link/sensor/imu_sensor/imu" | head -12
echo "--- 5 consecutive IMU stamps"
timeout 5 gz topic -e -t "/world/$WORLD/model/$MODEL_NAME/link/base_link/sensor/imu_sensor/imu" -n 5 | grep -A2 "stamp" | grep -E "sec|nsec" | paste - - | head -5
echo "--- GPU"
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv
uptime

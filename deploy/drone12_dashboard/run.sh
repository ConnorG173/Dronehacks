#!/usr/bin/env bash
# Start drone-conservative-v1 (dashboard mode) in a new tmux window "cv1" of the stack, plus a bag recorder "cv1bag".
#   bash run.sh <hover|first_leg|course> <base_height_on_pad_m>
# It only plans and waits: nothing moves until the PILOT arms in Offboard. Stop with stop.sh.
set -euo pipefail
source "$(dirname "$0")/common.sh"
PRESET="${1:?hover|first_leg|course}"; BH="${2:?base_height_on_pad_m (measured)}"
case "$PRESET" in hover|first_leg|course) ;; *) echo "preset must be hover, first_leg or course"; exit 1;; esac
if ct "$ENV_CT; ros2 node list 2>/dev/null | grep -q mission_tracker"; then echo "mission_tracker already running: stop.sh first"; exit 1; fi
T=$(date +%Y%m%d_%H%M%S)
ct "tmux kill-window -t $TMUX_S:cv1 2>/dev/null; tmux kill-window -t $TMUX_S:cv1bag 2>/dev/null; true"
ct "tmux new-window -d -t $TMUX_S -n cv1 \"bash -lc '$ENV_CT; ros2 launch team_path_following dashboard.launch.py repo_root:=$KIT_CT/repo mission_preset:=$PRESET base_height_on_pad_m:=$BH out_dir:=/root/team_ws/bags/cv1_${PRESET}_$T/plan 2>&1 | tee /root/team_ws/bags/cv1_${PRESET}_$T.log; bash'\""
ct "mkdir -p /root/team_ws/bags && tmux new-window -d -t $TMUX_S -n cv1bag \"bash -lc '$ENV_CT; ros2 bag record -o /root/team_ws/bags/cv1_${PRESET}_$T/bag /claude/vio_leveled/odometry /visual_slam/tracking/odometry /skybeta/setpoint/twist /skybeta/setpoint/pose /skybeta/state /team/mission/state /team/tracker/debug /team/plan/path /fmu/out/vehicle_local_position /tf /tf_static; bash'\""
echo "started: preset=$PRESET base_height=$BH  log ~/team_ws/bags/cv1_${PRESET}_$T.log"
echo "wait for 'plan ready' (15-60 s):  tail -f ~/team_ws/bags/cv1_${PRESET}_$T.log"

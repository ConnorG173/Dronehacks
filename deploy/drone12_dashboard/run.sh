#!/usr/bin/env bash
# Start drone-conservative-v1 (dashboard mode) in a new tmux window "cv1" of the stack, plus a bag recorder "cv1bag".
#   bash run.sh <hover|course> <base_height_on_pad_m> [route]
#   route: default = the changed course's route (path_following/routes/course_20261107_2212.bin, slowed to 0.86 m/s);
#          'gateb' = plan the old CAD Gate B route on the Jetson instead (first_leg only works with gateb)
# It only plans and waits: nothing moves until the PILOT arms in Offboard. Stop with stop.sh.
set -euo pipefail
source "$(dirname "$0")/common.sh"
PRESET="${1:?hover|course}"; BH="${2:?base_height_on_pad_m (measured)}"; ROUTE="${3:-course_20261107_2212}"
if [[ "$ROUTE" == gateb ]]; then RARG=""; else RARG="path_file:=$KIT_CT/repo/path_following/routes/$ROUTE.bin"; fi
if [[ -n "$RARG" && "$PRESET" == first_leg ]]; then echo "first_leg needs route gateb (it re-plans on the old CAD)"; exit 1; fi
case "$PRESET" in hover|first_leg|course) ;; *) echo "preset must be hover, first_leg or course"; exit 1;; esac
if ct "$ENV_CT; ros2 node list --no-daemon --spin-time 5 2>/dev/null | grep -q mission_tracker"; then echo "mission_tracker already running: stop.sh first"; exit 1; fi
POSE=$(px4_pose_topic)
echo "pose source = what PX4 fuses: $POSE"
if [[ "$POSE" == /team_heading/odometry ]]; then
  ct "$ENV_CT; timeout 10 ros2 topic echo --once --no-daemon /team_heading/status 2>/dev/null" | grep -q '"calibrated": true' \
    || echo "WARNING: heading fix not calibrated yet. Run calibrate.sh (drone still on the pad) before the pilot arms."
fi
T=$(date +%Y%m%d_%H%M%S)
ct "tmux kill-window -t $TMUX_S:cv1 2>/dev/null; tmux kill-window -t $TMUX_S:cv1bag 2>/dev/null; true"
ct "tmux new-window -d -t $TMUX_S -n cv1 \"bash -lc '$ENV_CT; ros2 launch team_path_following dashboard.launch.py repo_root:=$KIT_CT/repo mission_preset:=$PRESET base_height_on_pad_m:=$BH pose_topic:=$POSE $RARG out_dir:=/root/team_ws/bags/cv1_${PRESET}_$T/plan 2>&1 | tee /root/team_ws/bags/cv1_${PRESET}_$T.log; bash'\""
ct "mkdir -p /root/team_ws/bags && tmux new-window -d -t $TMUX_S -n cv1bag \"bash -lc '$ENV_CT; ros2 bag record -o /root/team_ws/bags/cv1_${PRESET}_$T/bag /claude/vio_leveled/odometry /team_heading/odometry /team_heading/status /visual_slam/tracking/odometry /skybeta/setpoint/twist /skybeta/setpoint/pose /skybeta/state /team/mission/state /team/tracker/debug /team/plan/path /fmu/out/vehicle_local_position /tf /tf_static; bash'\""
echo "started: preset=$PRESET route=$ROUTE pose=$POSE base_height=$BH  log ~/team_ws/bags/cv1_${PRESET}_$T.log"
echo "wait for 'plan ready' (15-60 s):  tail -f ~/team_ws/bags/cv1_${PRESET}_$T.log"

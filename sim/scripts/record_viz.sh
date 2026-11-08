#!/usr/bin/env bash
# Record this slot's Foxglove 3D view (course, drone, trails, plan, state) to ~/recordings/sim<N>_<time>.mcap.
# No camera images (too big). Open the .mcap in Foxglove with docs/foxglove_sim_layout.json.
set -eo pipefail
source "$(dirname "$0")/common.sh"
source /opt/ros/humble/setup.bash
mkdir -p ~/recordings
# --use-sim-time: log times = sim time, matching the message stamps (Foxglove playback timeline).
exec ros2 bag record -s mcap --use-sim-time -o ~/recordings/sim${SIM_ID}_$(date -u +%Y%m%d_%H%M%S) \
  /viz/course /viz/checkpoints /viz/drone /viz/trails /tf /tf_static \
  /sim/ground_truth/odom /visual_slam/tracking/odometry /team/plan/path /team/mission/state /team/odom_map /team/loc/status /clock

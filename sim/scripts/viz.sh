#!/usr/bin/env bash
# Foxglove 3D view publisher for this slot (course, checkpoints, drone, true vs VIO trails). See sim/tools/viz_course.py.
set -eo pipefail
source "$(dirname "$0")/common.sh"
source /opt/ros/humble/setup.bash
export REPO
# Sim time: the drone, TF and VIO are stamped from /clock; wall-clock stamps put the course ~30 years away in Foxglove.
exec python3 "$REPO/sim/tools/viz_course.py" --ros-args -p use_sim_time:=true

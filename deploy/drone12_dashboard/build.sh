#!/usr/bin/env bash
# Build the staged package + planner in the stack container, into the kit's own build/install dirs.
# Touches nothing outside ~/team_ws/team_staging/conservative_v1. Safe while the dashboard runs
# (CPU only: do NOT run while anyone is flying).
set -euo pipefail
source "$(dirname "$0")/common.sh"
ct "source skybeta_env.sh >/dev/null 2>&1; cd $KIT_CT &&
  colcon build --base-paths repo/path_following/team_path_following --build-base build --install-base install \
    --symlink-install --packages-select team_path_following 2>&1 | tail -3 &&
  cmake -S repo/planner -B repo/planner/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/ros/humble >/dev/null &&
  cmake --build repo/planner/build -j4 --target gcopter_planner 2>&1 | tail -2 &&
  ls -la repo/planner/build/gcopter_planner && source install/setup.bash && ros2 pkg executables team_path_following &&
  chown -R \${HOST_UID:-1000}:\${HOST_GID:-1000} $KIT_CT"

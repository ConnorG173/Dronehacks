#!/usr/bin/env bash
# Gazebo server for this slot, headless with GPU rendering (cameras + lidar render on the L4).
set -eo pipefail
source "$(dirname "$0")/common.sh"
gz_env
exec gz sim -r -s --headless-rendering -v 2 "$REPO/sim/gz/worlds/$WORLD.sdf"

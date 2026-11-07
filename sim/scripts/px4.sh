#!/usr/bin/env bash
# PX4 v1.16.2 SITL attached to the running Gazebo world; spawns the skybeta model at $POSE.
set -eo pipefail
source "$(dirname "$0")/common.sh"
gz_env
export PX4_GZ_MODELS="$REPO/sim/gz/models"
export PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$WORLD" PX4_SIM_MODEL=gz_skybeta PX4_GZ_MODEL_POSE="$POSE"
cd "$PX4_DIR/build/px4_sitl_default/rootfs"
exec ../bin/px4

#!/usr/bin/env bash
# PX4 v1.16.2 SITL instance $SIM_ID attached to this slot's Gazebo world; spawns the skybeta model at $POSE.
set -eo pipefail
source "$(dirname "$0")/common.sh"
gz_env
export PX4_GZ_MODELS="$REPO/sim/gz/models"
export PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$WORLD" PX4_SIM_MODEL=gz_skybeta PX4_GZ_MODEL_POSE="$POSE"
export PX4_UXRCE_DDS_PORT="$XRCE_PORT"
export PX4_NO_UXRCE_NS=1               # keep /fmu/* un-namespaced (slots are isolated by ROS domain instead)
WORK="$HOME/.px4_sim/$SIM_ID"
mkdir -p "$WORK" && cd "$WORK"
exec "$PX4_BUILD/bin/px4" -i "$SIM_ID" -d "$PX4_BUILD/etc"

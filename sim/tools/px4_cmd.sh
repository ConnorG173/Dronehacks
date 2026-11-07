#!/usr/bin/env bash
# Run a PX4 shell command against this slot's running PX4 (daemon mode, so no interactive console):
#   SIM_ID=0 bash sim/tools/px4_cmd.sh listener estimator_status_flags
#   SIM_ID=0 bash sim/tools/px4_cmd.sh commander check
#   SIM_ID=0 bash sim/tools/px4_cmd.sh param show EKF2_MAG_TYPE
source "$(dirname "$0")/../scripts/common.sh"
CMD="$1"; shift
cd "$HOME/.px4_sim/$SIM_ID"
exec "$PX4_BUILD/bin/px4-$CMD" --instance "$SIM_ID" "$@"

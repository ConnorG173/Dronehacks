#!/usr/bin/env bash
# Run a PX4 shell command in this slot's PX4 console and print the output:
#   SIM_ID=0 bash sim/tools/px4_cmd.sh "listener vehicle_local_position"
source "$(dirname "$0")/../scripts/common.sh"
tmux send-keys -t "$TMUX_SESSION:px4" "$1" Enter
sleep "${2:-2}"
tmux capture-pane -p -J -t "$TMUX_SESSION:px4" -S -60 | tail -"${3:-40}"

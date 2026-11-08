#!/usr/bin/env bash
# Stop our mission_tracker and recorder (Ctrl-C, then close the windows). The dashboard keeps running.
source "$(dirname "$0")/common.sh"
for w in cv1 cv1bag; do ct "tmux send-keys -t $TMUX_S:$w C-c 2>/dev/null; true"; done
sleep 3
for w in cv1 cv1bag; do ct "tmux kill-window -t $TMUX_S:$w 2>/dev/null; true"; done
ct "$ENV_CT; ros2 node list --no-daemon --spin-time 5 2>/dev/null | grep mission_tracker || echo 'mission_tracker stopped'"

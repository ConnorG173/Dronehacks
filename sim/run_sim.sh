#!/usr/bin/env bash
# Start the whole sim in tmux session "sim": gz | px4 | bridge | stack | foxglove.
#   bash sim/run_sim.sh            start (kills an old session first)
#   tmux attach -t sim             watch it (Ctrl-b n / p to switch windows, Ctrl-b d to detach)
#   bash sim/run_sim.sh stop       stop everything
# Then:  python3 sim/pilot.py go   in another shell, after `bash sim/scripts/competitor.sh`.
set -eo pipefail
S="$(cd "$(dirname "$0")" && pwd)/scripts"
tmux kill-session -t sim 2>/dev/null || true
docker rm -f skybeta-sim >/dev/null 2>&1 || true
pkill -f "gz sim" 2>/dev/null || true; pkill -x px4 2>/dev/null || true
[[ "${1:-}" == "stop" ]] && { echo "sim stopped"; exit 0; }
tmux new-session -d -s sim -n gz "bash $S/gz.sh; bash"
sleep 6
tmux new-window -t sim -n px4 "bash $S/px4.sh; bash"
sleep 6
tmux new-window -t sim -n bridge "bash $S/bridge.sh; bash"
tmux new-window -t sim -n stack "bash $S/stack.sh; bash"
tmux new-window -t sim -n foxglove "source /opt/ros/humble/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765; bash"
echo "sim starting in tmux session 'sim' (tmux attach -t sim)"

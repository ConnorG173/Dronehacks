#!/usr/bin/env bash
# Start one sim slot in tmux session "sim<N>": gz | px4 | bridge | stack | foxglove.
#   bash sim/run_sim.sh 0            start slot 0 (path following)   -- slot 1 = localization
#   bash sim/run_sim.sh 0 stop       stop slot 0 only
#   tmux attach -t sim0              watch (Ctrl-b n/p switch window, Ctrl-b d detach)
# Then: SIM_ID=0 bash sim/scripts/competitor.sh   and   python3 sim/pilot.py --sim 0 go
set -eo pipefail
export SIM_ID="${1:-${SIM_ID:-0}}"
S="$(cd "$(dirname "$0")" && pwd)/scripts"
source "$S/common.sh"
tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true   # SIGHUPs gz, bridge, foxglove in this slot
sleep 2
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
pkill -f "px4 -i $SIM_ID " 2>/dev/null || true
pkill -f "sim${SIM_ID}_bridge.yaml" 2>/dev/null || true
pkill -f "foxglove.*port:=$FOXGLOVE_PORT" 2>/dev/null || true
if [[ "${2:-}" == "stop" ]]; then echo "sim $SIM_ID stopped"; exit 0; fi
if [[ ! -f "$REPO/sim/gz/models/dronehacks_course/model.sdf" ]]; then
  echo "generating course model for this checkout ..."
  "$HOME/.venvs/simtools/bin/python" "$REPO/sim/tools/step_to_gz.py" "$REPO/dronehacks-course-v3.step" \
    --out "$REPO/sim/gz/models/dronehacks_course" | tail -1
fi
E="SIM_ID=$SIM_ID"
tmux new-session -d -s "$TMUX_SESSION" -n gz "$E bash $S/gz.sh; bash"
sleep 8
tmux new-window -t "$TMUX_SESSION" -n px4 "$E bash $S/px4.sh; bash"
sleep 8
tmux new-window -t "$TMUX_SESSION" -n bridge "$E bash $S/bridge.sh; bash"
tmux new-window -t "$TMUX_SESSION" -n stack "$E bash $S/stack.sh; bash"
tmux new-window -t "$TMUX_SESSION" -n foxglove "source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=$ROS_DOMAIN_ID ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=$FOXGLOVE_PORT; bash"
echo "sim $SIM_ID starting: tmux attach -t $TMUX_SESSION   (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, foxglove :$FOXGLOVE_PORT, MAVLink :$MAV_PORT)"

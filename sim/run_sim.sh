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
tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
# Killing tmux does NOT stop `gz sim` (it survives SIGHUP). A leftover server in the same partition makes
# PX4 see two interleaved clocks -> IMU "timestamp error" spam and no estimator. Kill this slot's processes
# by unique patterns and wait until they are really gone.
PATTERNS=("gz sim .*$REPO/sim/gz/worlds" "bin/px4 -i $SIM_ID " "sim${SIM_ID}_bridge.yaml" "foxglove.*port:=$FOXGLOVE_PORT" "sim/rc.py --sim $SIM_ID")
for p in "${PATTERNS[@]}"; do pkill -f "$p" 2>/dev/null || true; done
for _ in $(seq 1 20); do
  alive=0; for p in "${PATTERNS[@]}"; do pgrep -f "$p" >/dev/null && alive=1; done
  [[ $alive == 0 ]] && break; sleep 0.5
done
for p in "${PATTERNS[@]}"; do pkill -9 -f "$p" 2>/dev/null || true; done
if [[ "${2:-}" == "stop" ]]; then echo "sim $SIM_ID stopped"; exit 0; fi
if [[ ! -f "$REPO/sim/gz/models/dronehacks_course/model.sdf" ]]; then
  echo "generating course model for this checkout ..."
  "$HOME/.venvs/simtools/bin/python" "$REPO/sim/tools/step_to_gz.py" "$REPO/dronehacks-course-v3.step" \
    --out "$REPO/sim/gz/models/dronehacks_course" | tail -1
fi
if [[ ! -f "$REPO/sim/gz/models/skybeta_frame/model.sdf" ]]; then
  python3 "$REPO/sim/tools/make_skybeta_frame.py" --px4 "$PX4_DIR"
fi
E="SIM_ID=$SIM_ID"
tmux new-session -d -s "$TMUX_SESSION" -n gz "$E bash $S/gz.sh; bash"
sleep 8
tmux new-window -t "$TMUX_SESSION" -n px4 "$E bash $S/px4.sh; bash"
sleep 8
# PX4 attaches to an already-running Gazebo, and its first IMU samples are invalid (timestamp 0). EKF2
# initialises on them, diverges (tilt off by tens of degrees for ~30 s), and keeps a corrupted accel bias
# that never re-converges at rest -> "High Accelerometer Bias" blocks arming. Restart EKF2 once the
# sensors are clean; afterwards attitude and bias match ground truth.
tmux new-window -t "$TMUX_SESSION" -n ekfreset "sleep 30; $E bash $REPO/sim/tools/px4_cmd.sh ekf2 stop; sleep 1; $E bash $REPO/sim/tools/px4_cmd.sh ekf2 start; echo 'EKF2 restarted after sensor settle'; bash"
tmux new-window -t "$TMUX_SESSION" -n rc "python3 $REPO/sim/rc.py --sim $SIM_ID; bash"
tmux new-window -t "$TMUX_SESSION" -n bridge "$E bash $S/bridge.sh; bash"
tmux new-window -t "$TMUX_SESSION" -n stack "$E bash $S/stack.sh; bash"
tmux new-window -t "$TMUX_SESSION" -n foxglove "source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=$ROS_DOMAIN_ID FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=$FOXGLOVE_PORT; bash"
echo "sim $SIM_ID starting: tmux attach -t $TMUX_SESSION   (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, foxglove :$FOXGLOVE_PORT, MAVLink :$MAV_PORT)"

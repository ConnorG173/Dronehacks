#!/usr/bin/env bash
# Path-following runs in the Gazebo sim (docs/SIM.md), on the VM in slot 0's working copy.
# From the laptop:   bash sim/gcp/sync.sh 0
#                    bash sim/gcp/vm.sh 0 'bash path_following/sim/slot0.sh build'
#                    bash sim/gcp/vm.sh 0 'bash path_following/sim/slot0.sh fly gt run01'
#
#   build                         colcon-build team_path_following + the planner inside the stack container
#   fly <gt|vio> <name> [args]    launch flight interface + mission_tracker (tmux window 'team'), record a bag,
#                                 give the pilot's go, wait for done/abort (or 180 s), land, score the run
#                                 gt  = track ground truth (/sim/ground_truth/odom, map frame): tests the tracker
#                                 vio = track cuVSLAM odometry + launch-pose map->odom (the drone's fallback)
#                                 extra args go to the launch file, e.g. setpoint_rate_hz:=50.0
#   stop                          stop the team window and any bag recorder
set -eo pipefail
export SIM_ID=0
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
source "$REPO/sim/scripts/common.sh"
IN_C="source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && cd /root/team"
RUNS=path_following/runs/sim
COURSE=planner/config/course_v3.yaml

cmd="${1:-}"; shift || true
case "$cmd" in
build)
  docker exec "$CONTAINER" bash -lc "$IN_C && colcon build --symlink-install --packages-select team_path_following &&
    cmake -S planner -B planner/build -DCMAKE_BUILD_TYPE=Release >/dev/null && cmake --build planner/build -j8 --target gcopter_planner &&
    ls -la planner/build/gcopter_planner"
  ;;
fly)
  mode="${1:?gt|vio}"; name="${2:?run name}"; shift 2
  case "$mode" in
    gt)  POSE_ARGS="pose_topic:=/sim/ground_truth/odom pose_topic_frame:=map" ;;
    vio) POSE_ARGS="pose_topic:=/visual_slam/tracking/odometry pose_topic_frame:=odom" ;;
    *) echo "mode must be gt or vio"; exit 1 ;;
  esac
  out="$RUNS/$name"; mkdir -p "$REPO/$out"
  tmux kill-window -t "$TMUX_SESSION:team" 2>/dev/null || true
  tmux new-window -d -t "$TMUX_SESSION" -n team \
    "bash $REPO/sim/scripts/competitor.sh team_path_following mission.launch.py allow_mode_request:=true \
       $POSE_ARGS course_config:=/root/team/$COURSE planner_binary:=/root/team/planner/build/gcopter_planner \
       out_dir:=/root/team/$out/plan $* 2>&1 | tee $REPO/$out/launch.log"
  docker exec -d "$CONTAINER" bash -lc "$IN_C && ros2 bag record -s mcap -o /root/team/$out/bag \
    /sim/ground_truth/odom /visual_slam/tracking/odometry /skybeta/setpoint/twist /skybeta/state \
    /team/mission/state /team/tracker/debug /team/plan/path /fmu/in/trajectory_setpoint /fmu/out/vehicle_local_position \
    >/root/team/$out/bag.log 2>&1"
  echo "waiting for the plan (mission_tracker streams only once it has one)..."
  for _ in $(seq 1 120); do grep -q "plan ready" "$REPO/$out/launch.log" 2>/dev/null && break; sleep 1; done
  grep -q "plan ready" "$REPO/$out/launch.log" || { echo "no plan after 120 s"; tail -20 "$REPO/$out/launch.log"; exit 1; }
  python3 "$REPO/sim/pilot.py" --sim 0 go
  for _ in $(seq 1 180); do
    state=$(docker exec "$CONTAINER" bash -lc "$IN_C && timeout 3 ros2 topic echo --once /team/mission/state std_msgs/msg/String 2>/dev/null" | sed -n 's/^data: //p' | tr -d "'")
    echo -ne "\rmission: ${state:-?}      "
    [[ "$state" == "done" || "$state" == "abort" ]] && break
    sleep 1
  done; echo
  [[ "$state" == "done" ]] || python3 "$REPO/sim/pilot.py" --sim 0 land || true
  docker exec "$CONTAINER" pkill -INT -f "ros2 bag record" || true
  sleep 3
  docker exec "$CONTAINER" bash -lc "$IN_C && python3 path_following/tools/eval_bag.py $out/bag $COURSE $out/plan/path.bin \
    --name '$name ($mode $*)' --append docs/results/path_following.md && python3 path_following/tools/fit_lag.py $out/bag" | tee "$REPO/$out/metrics.txt"
  ;;
stop)
  tmux kill-window -t "$TMUX_SESSION:team" 2>/dev/null || true
  docker exec "$CONTAINER" pkill -INT -f "ros2 bag record" || true
  ;;
*) sed -n '2,15p' "$0"; exit 1 ;;
esac

#!/usr/bin/env bash
# Localization runs in the Gazebo sim, slot 1 (docs/SIM.md), on the VM in ~/slot1/Dronehacks.
# From the laptop (this worktree):
#   bash sim/gcp/sync.sh 1                    # this branch; path_following comes from its branch, see below
#   bash sim/gcp/vm.sh 1 'bash tools/loc_dev/slot1.sh build'
#   bash sim/gcp/vm.sh 1 'bash tools/loc_dev/slot1.sh fly gt run01'    # tracker on ground truth, loc passive
#   bash sim/gcp/vm.sh 1 'bash tools/loc_dev/slot1.sh fly loc run02'   # tracker on /team/odom_map (integration)
#
#   build                      colcon-build team_loc, team_loc_matcher, team_path_following + the planner
#   fly <gt|loc> <name> [args] start team_loc (correction + wall matcher; extra args go to team_loc.launch.py),
#                              path following's mission_tracker, a bag; pilot go; wait; land; loc_eval.py
#   reset                      fresh world + PX4 (drone on the launch mark), then check_sim
#   stop                       stop the loc/team windows and the bag recorder
# The tracker is path following's (branch path-following): copy path_following/ into the slot first.
set -eo pipefail
export SIM_ID=1
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
source "$REPO/sim/scripts/common.sh"
DDS=/root/team/sim/dds/fastdds_udp.xml
IN_C="source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && cd /root/team"
RUNS=runs_loc
COURSE=planner/config/course_v3.yaml

dexec() { docker exec -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e FASTRTPS_DEFAULT_PROFILES_FILE="$DDS" "$@"; }

start_loc() {
  # A runner script avoids nested quoting through tmux + docker exec.
  cat > "$REPO/$RUNS/loc_run.sh" <<EOF
#!/usr/bin/env bash
docker exec -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e FASTRTPS_DEFAULT_PROFILES_FILE=$DDS $CONTAINER bash -lc \
  "$IN_C && source install/setup.bash && ros2 launch team_loc team_loc.launch.py course_config:=/root/team/$COURSE enable_correction:=true with_matcher:=true $*" \
  2>&1 | tee $REPO/$RUNS/loc_latest.log
EOF
  chmod +x "$REPO/$RUNS/loc_run.sh"
  tmux kill-window -t "$TMUX_SESSION:loc" 2>/dev/null || true
  # stop a previous launch inside the container too (the tmux window only held docker exec)
  dexec "$CONTAINER" pkill -INT -f "team_loc.launch.py" 2>/dev/null || true
  sleep 1
  tmux new-window -d -t "$TMUX_SESSION" -n loc "bash $REPO/$RUNS/loc_run.sh; bash"
}

cmd="${1:-}"; shift || true
mkdir -p "$REPO/$RUNS"
case "$cmd" in
build)
  dexec "$CONTAINER" bash -lc "$IN_C && colcon build --symlink-install --packages-select team_loc team_loc_matcher team_path_following \
    --cmake-args -DBUILD_TESTING=OFF 2>&1 | tail -5"
  dexec "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && cd /root/team && cmake -S planner -B planner/build -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/opt/ros/humble >/dev/null && cmake --build planner/build -j8 --target gcopter_planner 2>&1 | tail -1"
  ;;
fly)
  mode="${1:?gt|loc}"; name="${2:?run name}"; shift 2
  case "$mode" in
    gt)  POSE_ARGS="pose_topic:=/sim/ground_truth/odom pose_topic_frame:=course" ;;
    loc) POSE_ARGS="" ;;
    *) echo "mode must be gt or loc"; exit 1 ;;
  esac
  out="$RUNS/$name"
  if [ -e "$REPO/$out" ]; then echo "run $name exists ($out): pick a new name"; exit 1; fi
  mkdir -p "$REPO/$out"
  start_loc "$@"
  sleep 6
  tmux kill-window -t "$TMUX_SESSION:team" 2>/dev/null || true
  tmux new-window -d -t "$TMUX_SESSION" -n team \
    "SIM_ID=1 bash $REPO/sim/scripts/competitor.sh team_path_following mission.launch.py allow_mode_request:=true \
       $POSE_ARGS course_config:=/root/team/$COURSE planner_binary:=/root/team/planner/build/gcopter_planner \
       out_dir:=/root/team/$out/plan 2>&1 | tee $REPO/$out/launch.log; bash"
  dexec -d "$CONTAINER" bash -lc "$IN_C && ros2 bag record -s mcap -o /root/team/$out/bag \
    /sim/ground_truth/odom /visual_slam/tracking/odometry /visual_slam/status /team/odom_map /team/loc/status \
    /team/loc/fix /team/loc/matcher_status /team/loc/correction /team/mission/state /team/plan/path \
    /skybeta/setpoint/twist /skybeta/state /fmu/out/vehicle_local_position /fmu/out/vehicle_attitude \
    /tf /tf_static >/root/team/$out/bag.log 2>&1"
  echo "waiting for the plan..."
  for _ in $(seq 1 120); do
    if grep -q "plan ready" "$REPO/$out/launch.log" 2>/dev/null; then break; fi
    sleep 1
  done
  grep -q "plan ready" "$REPO/$out/launch.log" || { echo "no plan after 120 s"; tail -20 "$REPO/$out/launch.log"; exit 1; }
  sleep 2
  echo "--- pilot go"
  timeout 150 python3 "$REPO/sim/pilot.py" --sim 1 go || echo "pilot go failed"
  ok=0
  python3 "$REPO/path_following/sim/wait_mission.py" 180 && ok=1
  if [ "$ok" != 1 ]; then timeout 30 python3 "$REPO/sim/pilot.py" --sim 1 land || true; fi
  dexec "$CONTAINER" pkill -INT -f "ros2 bag record" || true
  sleep 3
  cp "$REPO/$RUNS/loc_latest.log" "$REPO/$out/loc.log" || true
  dexec "$CONTAINER" bash -lc "$IN_C && python3 tools/loc_eval.py $out/bag --config $COURSE --out $out/eval --label '$name ($mode $*)'" \
    | tee "$REPO/$out/metrics.txt"
  grep -E "re-acquired|jump|recovered|anchored" "$REPO/$out/loc.log" | tail -5 || true
  ;;
reset)
  # fresh world + PX4, drone back on the launch mark. Gazebo sometimes comes up without camera
  # images: retry up to 3 times until /oak/left/image_mono flows.
  bash "$0" stop || true
  for attempt in 1 2 3; do
    bash "$REPO/sim/run_sim.sh" 1 stop || true
    sleep 3
    bash "$REPO/sim/run_sim.sh" 1
    echo "settling 150 s (attempt $attempt; Gazebo loads the textured course in ~2 min, EKF2 restarts after PX4)..."
    sleep 150
    bash "$REPO/sim/tools/check_sim.sh" > "$REPO/$RUNS/check_sim.txt" 2>&1 || true
    grep -iE "rate|xy_valid|armed|blank|image|FAIL" "$REPO/$RUNS/check_sim.txt" | head -10
    if grep -q "/oak/left/image_mono: average rate" "$REPO/$RUNS/check_sim.txt" && grep -q "xy_valid: true" "$REPO/$RUNS/check_sim.txt"; then
      echo "sim healthy"; break
    fi
    echo "sim not healthy, restarting"
  done
  ;;
stop)
  tmux kill-window -t "$TMUX_SESSION:team" 2>/dev/null || true
  tmux kill-window -t "$TMUX_SESSION:loc" 2>/dev/null || true
  dexec "$CONTAINER" pkill -INT -f "team_loc.launch.py" || true
  dexec "$CONTAINER" pkill -INT -f "mission.launch.py" || true
  dexec "$CONTAINER" pkill -INT -f "ros2 bag record" || true
  ;;
*) sed -n '2,15p' "$0"; exit 1 ;;
esac

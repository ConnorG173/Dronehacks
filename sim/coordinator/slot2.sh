#!/usr/bin/env bash
# Path-following runs in the Gazebo sim (docs/SIM.md), on the VM in slot 0's working copy.
# From the laptop:   bash sim/gcp/sync.sh 0
#                    bash sim/gcp/vm.sh 0 'bash path_following/sim/slot2.sh build'
#                    bash sim/gcp/vm.sh 0 'bash path_following/sim/slot2.sh fly gt run01'
#
#   build                         colcon-build team_path_following + the planner inside the stack container
#   fly <gt|vio|loc> <name> [args]  launch flight interface + mission_tracker (tmux window 'team'), record a
#                                 bag, give the pilot's go, wait for done/abort (or 180 s), land, score the run
#                                 gt  = track ground truth (/sim/ground_truth/odom, course frame): tracker only
#                                 vio = track cuVSLAM odometry + launch-pose course->odom (the drone's fallback)
#                                 loc = launch defaults: /team/odom_map + TF course->odom (needs team_loc running)
#                                 extra args go to the launch file, e.g. setpoint_rate_hz:=50.0 speed_factor:=1.1
#                                 LAUNCH=conservative.launch.py flies the frozen drone profile (fixes its own course/gains)
#   reset                         restart the sim (fresh world + PX4) and wait for it to settle
#   stop                          stop the team window and any bag recorder
set -eo pipefail
export SIM_ID=2
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
source "$REPO/sim/scripts/common.sh"
DDS=/root/team/sim/dds/fastdds_udp.xml
IN_C="source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && cd /root/team"
RUNS=path_following/runs/sim
COURSE="${COURSE:-planner/config/course_v3_gateb.yaml}"   # COURSE=planner/config/course_v3.yaml: window route
CP_VIA="${CP_VIA-5,6}"   # routing-aid checkpoints in course_v3_gateb (not scored as course checkpoints); "" for course_v3

dexec() { docker exec -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e FASTRTPS_DEFAULT_PROFILES_FILE="$DDS" "$@"; }

cmd="${1:-}"; shift || true
case "$cmd" in
build)
  dexec "$CONTAINER" bash -lc "$IN_C && colcon build --symlink-install --packages-select team_path_following"
  dexec "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && cd /root/team && cmake -S planner -B planner/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/ros/humble >/dev/null && cmake --build planner/build -j8 --target gcopter_planner && ls -la planner/build/gcopter_planner"
  ;;
fly)
  mode="${1:?gt|vio|loc}"; name="${2:?run name}"; shift 2
  case "$mode" in
    gt)  POSE_ARGS="pose_topic:=/sim/ground_truth/odom pose_topic_frame:=course" ;;
    vio) POSE_ARGS="pose_topic:=/visual_slam/tracking/odometry pose_topic_frame:=odom" ;;
    loc) POSE_ARGS="" ;;
    *) echo "mode must be gt, vio or loc"; exit 1 ;;
  esac
  out="$RUNS/$name"
  mkdir -p "$REPO/$out"
  tmux kill-window -t "$TMUX_SESSION:team" 2>/dev/null || true
  tmux new-window -d -t "$TMUX_SESSION" -n team \
    "SIM_ID=2 bash $REPO/sim/scripts/competitor.sh team_path_following ${LAUNCH:-mission.launch.py} allow_mode_request:=true tracker_use_sim_time:=${SIM_TIME:-true} \
       $POSE_ARGS mission_preset:=course course_config:=/root/team/$COURSE planner_binary:=/root/team/planner/build/gcopter_planner \
       out_dir:=/root/team/$out/plan $* 2>&1 | tee $REPO/$out/launch.log; bash"
  dexec -d "$CONTAINER" bash -lc "$IN_C && ros2 bag record -s mcap -o /root/team/$out/bag \
    /sim/ground_truth/odom /visual_slam/tracking/odometry /team/odom_map /team/loc/status /skybeta/setpoint/twist \
    /skybeta/state /team/mission/state /team/tracker/debug /team/plan/path /fmu/in/trajectory_setpoint \
    /fmu/out/vehicle_local_position /tf /tf_static >/root/team/$out/bag.log 2>&1"
  echo "waiting for the plan (mission_tracker streams only once it has one)..."
  for _ in $(seq 1 120); do
    if grep -q "plan ready" "$REPO/$out/launch.log" 2>/dev/null; then break; fi
    sleep 1
  done
  if ! grep -q "plan ready" "$REPO/$out/launch.log"; then
    echo "no plan after 120 s"; tail -20 "$REPO/$out/launch.log"; exit 1
  fi
  sleep 2
  # Gazebo real-time factor during the flight: a starved VM (RTF << 1) makes wall-clock tracking look laggy
  ( while :; do GZ_PARTITION=$GZ_PARTITION timeout 6 gz topic -e -n 1 -t /stats 2>/dev/null \
      | awk '/real_time_factor/ {print $2}' >> "$REPO/$out/rtf.txt"; sleep 4; done ) &
  RTF_PID=$!
  echo "--- pilot go"
  timeout 150 python3 "$REPO/sim/pilot.py" --sim 2 go || echo "pilot go failed"
  ok=0
  python3 "$REPO/path_following/sim/wait_mission.py" 180 && ok=1
  if [ "$ok" != 1 ]; then timeout 30 python3 "$REPO/sim/pilot.py" --sim 2 land || true; fi
  dexec "$CONTAINER" pkill -INT -f "ros2 bag record" || true
  kill $RTF_PID 2>/dev/null || true
  python3 -c "import sys; v=[float(x) for x in open(sys.argv[1]) if x.strip()]; print(f'gazebo RTF during flight: mean {sum(v)/len(v):.2f} min {min(v):.2f} ({len(v)} samples)' + ('  ** RUN INVALID FOR TIMING (RTF < 0.9) **' if sum(v)/len(v) < 0.9 else '')) if v else print('gazebo RTF: no samples')" "$REPO/$out/rtf.txt" 2>/dev/null | tee "$REPO/$out/rtf_summary.txt"
  sleep 3
  dexec "$CONTAINER" bash -lc "$IN_C && python3 path_following/tools/eval_bag.py $out/bag $COURSE $out/plan/path.bin \
    --name '$name' --notes '$mode $*' --via '$CP_VIA' --append $out/results_row.md; python3 path_following/tools/fit_lag.py $out/bag; python3 path_following/tools/vio_err.py $out/bag" \
    | tee "$REPO/$out/metrics.txt"
  ;;
sysid)
  # velocity-step flight for fit_lag.py:  sysid <name> <height_m> [amp_mps] [extra sysid_node args]
  # (above the walls does not work: the rangefinder height reference sees the wall tops)
  name="${1:?run name}"; height="${2:?height}"; amp="${3:-0.8}"; shift 3 || shift $#
  out="$RUNS/$name"
  mkdir -p "$REPO/$out"
  tmux kill-window -t "$TMUX_SESSION:team" 2>/dev/null || true
  tmux new-window -d -t "$TMUX_SESSION" -n team \
    "SIM_ID=2 bash $REPO/sim/scripts/competitor.sh competitor_example competitor.launch.py competitor:=false allow_mode_request:=true 2>&1 | tee $REPO/$out/launch.log; bash"
  dexec -d "$CONTAINER" bash -lc "$IN_C && python3 path_following/tools/sysid_node.py $COURSE --height $height --amp $amp $* >/root/team/$out/sysid.log 2>&1"
  dexec -d "$CONTAINER" bash -lc "$IN_C && ros2 bag record -s mcap -o /root/team/$out/bag /sim/ground_truth/odom \
    /skybeta/setpoint/twist /skybeta/state /team/mission/state /team/tracker/debug /fmu/in/trajectory_setpoint \
    /fmu/out/vehicle_local_position >/root/team/$out/bag.log 2>&1"
  sleep 8
  # Gazebo real-time factor during the flight: a starved VM (RTF << 1) makes wall-clock tracking look laggy
  ( while :; do GZ_PARTITION=$GZ_PARTITION timeout 6 gz topic -e -n 1 -t /stats 2>/dev/null \
      | awk '/real_time_factor/ {print $2}' >> "$REPO/$out/rtf.txt"; sleep 4; done ) &
  RTF_PID=$!
  echo "--- pilot go"
  timeout 150 python3 "$REPO/sim/pilot.py" --sim 2 go || echo "pilot go failed"
  ok=0
  python3 "$REPO/path_following/sim/wait_mission.py" 120 && ok=1
  if [ "$ok" != 1 ]; then timeout 30 python3 "$REPO/sim/pilot.py" --sim 2 land || true; fi
  dexec "$CONTAINER" pkill -INT -f "ros2 bag record" || true
  dexec "$CONTAINER" pkill -INT -f "sysid_node.py" || true
  sleep 3
  dexec "$CONTAINER" bash -lc "$IN_C && python3 path_following/tools/fit_lag.py $out/bag --states course" | tee "$REPO/$out/metrics.txt"
  ;;
reset)
  # fresh world + PX4. Gazebo loads the course textures (~2 min), EKF2 restarts itself at PX4 up + 30 s:
  # wait 150 s, then until check_sim reports image contrast OK and xy_valid (at most 3 more minutes)
  tmux kill-window -t "$TMUX_SESSION:team" 2>/dev/null || true
  bash "$REPO/sim/run_sim.sh" 0 stop || true
  sleep 3
  bash "$REPO/sim/run_sim.sh" 0
  echo "settling 150 s..."
  sleep 150
  for _ in $(seq 1 9); do
    chk=$(bash "$REPO/sim/tools/check_sim.sh" 2>&1 || true)
    if echo "$chk" | grep -q "^OK" && echo "$chk" | grep -q "xy_valid: true"; then break; fi
    sleep 20
  done
  echo "$chk" | grep -E "rate|xy_valid|armed|^OK|^FAIL" | head -10
  # ground truth is what every run is scored against: once, a restart came up without it (cons01/02)
  if ! dexec "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic echo --once /sim/ground_truth/odom >/dev/null 2>&1"; then
    echo "no /sim/ground_truth/odom: restarting the sim once more"
    bash "$REPO/sim/run_sim.sh" 0 stop || true; sleep 3; bash "$REPO/sim/run_sim.sh" 0; sleep 180
    dexec "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic echo --once /sim/ground_truth/odom >/dev/null 2>&1" \
      && echo "ground truth OK" || echo "WARNING: still no ground truth (runs cannot be scored)"
  else
    echo "ground truth OK"
  fi
  ;;
stop)
  tmux kill-window -t "$TMUX_SESSION:team" 2>/dev/null || true
  dexec "$CONTAINER" pkill -INT -f "ros2 bag record" || true
  ;;
*) sed -n '2,16p' "$0"; exit 1 ;;
esac

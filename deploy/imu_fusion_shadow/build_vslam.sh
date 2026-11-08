#!/bin/bash
# Build the PATCHED cuVSLAM node (Isaac ROS visual_slam 3.2.6 + our IMU-timestamp fix) for the shadow.
# Run on the Jetson HOST as dronehacks:
#   bash ~/team_ws/team_staging/imu_fusion/build_vslam.sh          start (returns at once; runs unattended)
#   bash ~/team_ws/team_staging/imu_fusion/build_vslam.sh status   RUNNING / PAUSED / DONE / FAILED + log tail
#   bash ~/team_ws/team_staging/imu_fusion/build_vslam.sh stop     abort the build
# Why: the stock 3.2.6 node registers every IMU sample with the camera frame's timestamp, so its IMU fusion
# diverges (Gazebo: 4.6 m error vs 2 cm without fusion). See vio/patches/ on branch imu-fusion.
# How: inside skybeta-stack (docker exec -d: survives ssh/Wi-Fi loss; no network needed, the source is
# vslam_src.tgz = tag v3.2-15 with the patch applied) it builds ONLY that package into ./vio_ws/install, at
# nice 19 on 2 cores. build_guard.py FREEZES the build whenever PX4 reports armed / not landed / stale
# telemetry and resumes it after 10 s disarmed + landed. Nothing in /opt/ros, ~/skybeta or the running flight
# processes is touched; the flight cuVSLAM keeps the stock binary. ~6-10 min of CPU on the Orin Nano.
set -o pipefail
KIT=/root/team_ws/team_staging/imu_fusion
HOSTKIT="$(cd "$(dirname "$0")" && pwd)"
C=skybeta-stack
# TEST_CONTAINER=<name>: exercise this script in a throwaway container (VM), skipping the PX4 pre-check.
# Never set it on the drone.
[ -n "${TEST_CONTAINER:-}" ] && C="$TEST_CONTAINER"
ST="$HOSTKIT/vio_ws_build.status"
ENVS="source /opt/ros/humble/setup.bash; source /root/skybeta_ws/install/local_setup.bash"

case "${1:-start}" in
status)
  cat "$ST" 2>/dev/null || echo "never started"
  tail -3 "$HOSTKIT/vio_ws_build.log" 2>/dev/null; tail -2 "$HOSTKIT/vio_ws_guard.log" 2>/dev/null
  exit 0 ;;
stop)
  docker exec "$C" bash -c "pkill -f build_guard.py; P=\$(cat $KIT/vio_ws_build.pgid 2>/dev/null); [ -n \"\$P\" ] && kill -CONT -\$P && kill -TERM -\$P" 2>/dev/null
  echo "FAILED stopped by user $(date '+%F %T')" > "$ST"; echo stopped; exit 0 ;;
start) ;;
*) sed -n 2,6p "$0"; exit 1 ;;
esac

grep -q '^\(RUNNING\|PAUSED\)' "$ST" 2>/dev/null && { echo "a build is already running:"; cat "$ST"; exit 1; }
if [ -z "${TEST_CONTAINER:-}" ]; then
  docker exec "$C" bash -c "$ENVS; python3 $KIT/team_imu_log.py --check" \
    || { echo "REFUSING to build: drone must be disarmed and landed"; exit 1; }
fi
V=$(docker exec "$C" bash -c "grep -o '<version>[^<]*' /opt/ros/humble/share/isaac_ros_visual_slam/package.xml | cut -d'>' -f2")
[ "$V" = 3.2.6 ] || { echo "installed visual_slam is $V, the patch is for 3.2.6: stop"; exit 1; }
echo "STARTING $(date '+%F %T')" > "$ST"

# Everything below runs detached inside the container: the compile in its own process group (setsid), the
# guard next to it; whichever ends first, the build is never left running unguarded.
docker exec -d "$C" bash -c "$ENVS
  cd $KIT
  setsid bash -c 'set -e; mkdir -p vio_ws/src && cd vio_ws && rm -rf src/isaac_ros_visual_slam build install
    tar xzf $KIT/vslam_src.tgz -C src
    grep -q \"DroneHacks fix\" src/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp
    export MAKEFLAGS=-j2
    if command -v colcon >/dev/null; then
      nice -n 19 colcon build --base-paths src/isaac_ros_visual_slam --packages-select isaac_ros_visual_slam \
        --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    else
      nice -n 19 cmake -S src/isaac_ros_visual_slam -B build/isaac_ros_visual_slam -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=$KIT/vio_ws/install/isaac_ros_visual_slam
      nice -n 19 cmake --build build/isaac_ros_visual_slam -j2 && cmake --install build/isaac_ros_visual_slam
    fi
    test -f install/isaac_ros_visual_slam/lib/isaac_ros_visual_slam/isaac_ros_visual_slam' > $KIT/vio_ws_build.log 2>&1 &
  BP=\$!
  echo \$BP > $KIT/vio_ws_build.pgid
  python3 -u $KIT/build_guard.py --pgid \$BP --status $KIT/vio_ws_build.status > $KIT/vio_ws_guard.log 2>&1 &
  GP=\$!
  while kill -0 \$BP 2>/dev/null; do     # guard gone while the build lives -> kill the build
    kill -0 \$GP 2>/dev/null || { kill -CONT -\$BP; kill -TERM -\$BP; echo \"FAILED \$(date '+%F %T') guard exited, build killed\" > $KIT/vio_ws_build.status; exit 1; }
    sleep 2
  done
  wait \$BP; RC=\$?
  kill \$GP 2>/dev/null
  if [ \$RC = 0 ]; then echo \"DONE \$(date '+%F %T') patched build OK\" > $KIT/vio_ws_build.status
  else echo \"FAILED \$(date '+%F %T') rc \$RC (see vio_ws_build.log)\" > $KIT/vio_ws_build.status; fi"
sleep 3
echo "build launched (detached; safe to disconnect)."
cat "$ST"
echo "check later:  bash $HOSTKIT/build_vslam.sh status"

#!/bin/bash
# Build the PATCHED cuVSLAM node (Isaac ROS visual_slam 3.2.6 + our IMU-timestamp fix) for the shadow, on the
# Jetson HOST as dronehacks:   bash ~/team_ws/team_staging/imu_fusion/build_vslam.sh
# Why: the stock 3.2.6 node registers every IMU sample with the camera frame's timestamp, so its IMU fusion
# diverges (Gazebo: 4.6 m error vs 2 cm without fusion). See vio/patches/ on branch imu-fusion.
# What it does: inside skybeta-stack (docker exec), unpacks vslam_src.tgz (source of tag v3.2-15 with the
# patch applied) into ./vio_ws/src and builds ONLY that package into ./vio_ws/install, at nice 19 on 2 cores.
# Nothing in /opt/ros, ~/skybeta or the running flight processes is touched; the flight cuVSLAM keeps using
# the stock binary. Refuses unless PX4 is disarmed and landed. ~10-15 min on the Orin Nano.
set -o pipefail
KIT=/root/team_ws/team_staging/imu_fusion
HOSTKIT="$(cd "$(dirname "$0")" && pwd)"
C=skybeta-stack
docker exec "$C" bash -c "source /opt/ros/humble/setup.bash; source /root/skybeta_ws/install/local_setup.bash; python3 $KIT/team_imu_log.py --check" \
  || { echo "REFUSING to build: drone must be disarmed and landed"; exit 1; }
V=$(docker exec "$C" bash -c "grep -o '<version>[^<]*' /opt/ros/humble/share/isaac_ros_visual_slam/package.xml | cut -d'>' -f2")
[ "$V" = 3.2.6 ] || { echo "installed visual_slam is $V, the patch is for 3.2.6: stop"; exit 1; }
echo "building patched visual_slam 3.2.6 (log: $HOSTKIT/vio_ws_build.log) ..."
docker exec "$C" bash -c "set -e; source /opt/ros/humble/setup.bash; source /root/skybeta_ws/install/local_setup.bash
  mkdir -p $KIT/vio_ws/src && cd $KIT/vio_ws && rm -rf src/isaac_ros_visual_slam && tar xzf $KIT/vslam_src.tgz -C src
  grep -q 'DroneHacks fix' src/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp
  export MAKEFLAGS=-j2
  if command -v colcon >/dev/null; then
    nice -n 19 colcon build --base-paths src/isaac_ros_visual_slam --packages-select isaac_ros_visual_slam \
      --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  else
    nice -n 19 cmake -S src/isaac_ros_visual_slam -B build/isaac_ros_visual_slam -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=$KIT/vio_ws/install/isaac_ros_visual_slam
    nice -n 19 cmake --build build/isaac_ros_visual_slam -j2 && cmake --install build/isaac_ros_visual_slam
  fi" > "$HOSTKIT/vio_ws_build.log" 2>&1
rc=$?
tail -5 "$HOSTKIT/vio_ws_build.log"
B="$HOSTKIT/vio_ws/install/isaac_ros_visual_slam/lib/isaac_ros_visual_slam/isaac_ros_visual_slam"
if [ $rc = 0 ] && [ -f "$B" ]; then echo "OK: patched build at $B"; else echo "BUILD FAILED (rc $rc): see $HOSTKIT/vio_ws_build.log"; exit 1; fi

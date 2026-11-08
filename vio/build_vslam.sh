#!/usr/bin/env bash
# Build Isaac ROS visual_slam 3.2.6 (tag v3.2-15, the exact source of the organizers' binary) with our IMU
# timestamp fix into an overlay workspace. Run INSIDE the organizers' container (x86 sim image or the
# drone's skybeta image): the overlay links against that image's NITROS/GXF/cuVSLAM.
#   bash vio/build_vslam.sh [workspace=<repo>/vio_ws]
# Then `source <ws>/install/setup.bash` after the image's own setup files: libvisual_slam_node.so and the
# isaac_ros_visual_slam executable resolve to the patched build. Nothing in /opt/ros is modified.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="${1:-$HERE/../vio_ws}"
TAG=v3.2-15
mkdir -p "$WS/src"; cd "$WS/src"
if [ ! -d isaac_ros_visual_slam_src ]; then
  git clone -q --depth 1 --branch "$TAG" --filter=blob:none --sparse https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git isaac_ros_visual_slam_src
  git -C isaac_ros_visual_slam_src sparse-checkout set isaac_ros_visual_slam
  git -C isaac_ros_visual_slam_src apply "$HERE/patches/isaac_ros_visual_slam-3.2.6-imu-timestamps.patch"
fi
grep -q "DroneHacks fix" isaac_ros_visual_slam_src/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp
source /opt/ros/humble/setup.bash
[ -f /root/skybeta_ws/install/setup.bash ] && source /root/skybeta_ws/install/setup.bash
cd "$WS"
colcon build --base-paths src/isaac_ros_visual_slam_src/isaac_ros_visual_slam --packages-select isaac_ros_visual_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF ${COLCON_EXTRA:-}
echo "built: source $WS/install/setup.bash"

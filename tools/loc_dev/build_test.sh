#!/usr/bin/env bash
# Build and unit-test the localization packages offline, in a ROS 2 Humble container.
#
#   bash tools/loc_dev/build_test.sh            # build + test team_loc
#   bash tools/loc_dev/build_test.sh --shell    # same, then an interactive shell in the workspace
#
# Dependencies that the drone's image already has are built once into a cache OUTSIDE this repo
# (default ~/.cache/team_loc_dev), so nothing here can shadow the drone's own px4_msgs:
#   - px4_msgs, branch release/1.16 (PX4 v1.16.2 on the FC)
#   - a stub isaac_ros_visual_slam_interfaces with only VisualSlamStatus (fields of Isaac ROS 3.2)
# The repo is mounted read-only; build/install/log go to the cache.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/../.." && pwd)
CACHE=${TEAM_LOC_DEV_CACHE:-$HOME/.cache/team_loc_dev}
IMAGE=${TEAM_LOC_DEV_IMAGE:-team-loc-dev:humble}
mkdir -p "$CACHE/deps_src"

if [ ! -d "$CACHE/deps_src/px4_msgs" ]; then
  git clone --depth 1 -b release/1.16 https://github.com/PX4/px4_msgs.git "$CACHE/deps_src/px4_msgs"
fi

STUB="$CACHE/deps_src/isaac_ros_visual_slam_interfaces"
if [ ! -f "$STUB/package.xml" ]; then
  mkdir -p "$STUB/msg"
  cat > "$STUB/msg/VisualSlamStatus.msg" <<'MSG'
# Stub of isaac_ros_visual_slam_interfaces/VisualSlamStatus (Isaac ROS 3.2), for offline builds only.
std_msgs/Header header
uint8 vo_state
float64 node_callback_execution_time
float64 track_execution_time
float64 track_execution_time_mean
float64 track_execution_time_max
MSG
  cat > "$STUB/package.xml" <<'XML'
<?xml version="1.0"?>
<package format="3">
  <name>isaac_ros_visual_slam_interfaces</name>
  <version>0.0.0</version>
  <description>Offline-build stub (VisualSlamStatus only). Never install on the drone.</description>
  <maintainer email="none@example.com">stub</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type></export>
</package>
XML
  cat > "$STUB/CMakeLists.txt" <<'CM'
cmake_minimum_required(VERSION 3.8)
project(isaac_ros_visual_slam_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/VisualSlamStatus.msg" DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
CM
fi

SHELL_AFTER=0
TTY=()
if [ "${1:-}" = "--shell" ]; then SHELL_AFTER=1; TTY=(-it); fi

docker run --rm ${TTY[@]+"${TTY[@]}"} -v "$REPO":/repo:ro -v "$CACHE":/cache -e SHELL_AFTER=$SHELL_AFTER "$IMAGE" bash -c '
set -e
source /opt/ros/humble/setup.bash
if [ ! -f /cache/deps_install/setup.bash ]; then
  echo "== building deps (once): px4_msgs, isaac interfaces stub"
  cd /cache && colcon --log-base /cache/deps_log build --base-paths /cache/deps_src \
     --build-base /cache/deps_build --install-base /cache/deps_install \
     --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --event-handlers console_cohesion- >/dev/null
fi
source /cache/deps_install/setup.bash
cd /cache
echo "== building team_loc"
colcon --log-base /cache/log build --base-paths /repo/src --build-base /cache/build --install-base /cache/install \
   --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ 2>&1 | grep -E "warning|error|Error|Finished|Summary|Failed" || true
echo "== testing"
colcon --log-base /cache/log test --base-paths /repo/src --build-base /cache/build --install-base /cache/install \
   --event-handlers console_direct- >/dev/null || true
colcon test-result --test-result-base /cache/build --verbose | tail -25
if [ "$SHELL_AFTER" = 1 ]; then source /cache/install/setup.bash; exec bash; fi
'

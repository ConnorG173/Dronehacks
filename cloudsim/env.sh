# Source on the cloud sim VM before any ROS command:   source ~/Dronehacks/cloudsim/env.sh
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
# The simulator-only guard in localizer.sim (--i-have-team-approval). Legitimate here because this
# VM is an isolated simulator with no flight controller. NEVER set these on the drone's Jetson.
export FLYX_LAB_ACTIVE=1 FLYX_HITL=0

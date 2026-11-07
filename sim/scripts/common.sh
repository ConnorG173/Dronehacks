# Shared settings for the sim scripts. Override any of these in the environment.
REPO="${REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
SKYBETA_DIR="${SKYBETA_DIR:-$HOME/skybeta}"
WORLD="${WORLD:-dronehacks_course}"
# Spawn pose x,y,z,roll,pitch,yaw in the map frame: the launch mark (planner StartPos), facing +x.
POSE="${POSE:--2.5146,0.9144,0.25,0,0,0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
gz_env() {
  # PX4's Gazebo environment (server.config with sensor systems, PX4 models/plugins), plus ours.
  source "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
  export GZ_SIM_RESOURCE_PATH="$REPO/sim/gz/models:$REPO/sim/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
}

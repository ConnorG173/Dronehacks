# Shared settings for the sim scripts. Two isolated sims can run side by side on one VM:
#   SIM_ID=0 (path following)   SIM_ID=1 (localization)
# Everything that could collide is derived from SIM_ID: Gazebo partition, PX4 instance, ROS domain,
# uXRCE-DDS port, MAVLink port, container name, Foxglove port, tmux session.
SIM_ID="${SIM_ID:-0}"
REPO="${REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
SKYBETA_DIR="${SKYBETA_DIR:-$HOME/skybeta}"
WORLD="${WORLD:-dronehacks_course}"
# Spawn pose x,y,z,roll,pitch,yaw in the map frame: the launch mark (planner StartPos), facing +x.
CAMERA_X=0.12; CAMERA_Z=0.242   # stereo rig in base_link (sim/gz/models/skybeta/model.sdf oak_link)
POSE="${POSE:--2.5146,0.9144,0.0,0,0,0}"   # z=0: rest on the floor; a drop at spawn corrupts EKF2 tilt alignment

export ROS_DOMAIN_ID="$((10 + SIM_ID))"
export GZ_PARTITION="sim${SIM_ID}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO/sim/dds/fastdds_udp.xml"   # see that file: no SHM across the container boundary
XRCE_PORT="$((8888 + SIM_ID))"
MAV_PORT="$((18570 + SIM_ID))"          # PX4's GCS link for this instance (sim/pilot.py talks here)
FOXGLOVE_PORT="$((8765 + SIM_ID))"
CONTAINER="skybeta-sim-${SIM_ID}"
MODEL_NAME="skybeta_${SIM_ID}"          # PX4 names the spawned model <model>_<instance>
TMUX_SESSION="sim${SIM_ID}"

gz_env() {
  # PX4's Gazebo environment (server.config with sensor systems, PX4 models/plugins), plus ours.
  source "$PX4_BUILD/rootfs/gz_env.sh"
  export GZ_SIM_RESOURCE_PATH="$REPO/sim/gz/models:$REPO/sim/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
}

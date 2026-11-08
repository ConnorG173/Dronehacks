# Shared settings for the drone-12 dashboard kit. Host paths; the container sees ~/team_ws as /root/team_ws.
KIT_HOST="$HOME/team_ws/team_staging/conservative_v1"
KIT_CT="/root/team_ws/team_staging/conservative_v1"
CT=skybeta-stack
TMUX_S=skybeta
ENV_CT="source skybeta_env.sh >/dev/null 2>&1; source $KIT_CT/install/setup.bash"
ct() { docker exec "$CT" bash -lc "$1"; }
# The pose PX4 fuses = the px4_odom_bridge input (levelled VIO, or the gyro heading fix when its marker is set).
# Our tracker must steer on exactly that topic, or its velocity commands rotate against PX4's frame.
px4_pose_topic() {
  local t; t=$(ct "$ENV_CT; timeout 20 ros2 param get --no-daemon /px4_odom_bridge input_topic 2>/dev/null" | sed -n 's/^String value is: //p')
  if [[ -z "$t" ]]; then
    if ct "test -e /root/team_ws/claude/mavlink_usb/heading_fix_enabled"; then t=/team_heading/odometry; else t=/claude/vio_leveled/odometry; fi
    echo "(bridge param unreadable; marker says $t)" >&2
  fi
  echo "$t"
}

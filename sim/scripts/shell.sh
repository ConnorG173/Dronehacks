#!/usr/bin/env bash
# A shell in this slot's ROS domain: inside the stack container (default) or on the host (--host).
set -eo pipefail
source "$(dirname "$0")/common.sh"
if [[ "${1:-}" == "--host" ]]; then
  exec bash --rcfile <(echo "source ~/.bashrc; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION; PS1='[sim$SIM_ID host] \w\$ '")
fi
exec docker exec -it -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && cd /root/team && exec bash"

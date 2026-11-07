#!/usr/bin/env bash
# From your laptop: run a command in slot N's working copy on the VM, with that slot's ROS domain.
#   bash sim/gcp/vm.sh 0 'bash sim/run_sim.sh 0'
#   bash sim/gcp/vm.sh 0 'python3 sim/pilot.py --sim 0 go'
#   bash sim/gcp/vm.sh 0 'tmux capture-pane -p -t sim0:px4 | tail -30'
set -eo pipefail
SLOT="${1:?usage: vm.sh <slot> <command>}"; shift
source "$(cd "$(dirname "$0")" && pwd)/vm.env"
gcloud compute ssh "$VM" --project "$PROJECT" --account "$ACCOUNT" --zone "$ZONE" --quiet -- -t \
  "cd ~/slot$SLOT/Dronehacks && export SIM_ID=$SLOT ROS_DOMAIN_ID=$((10 + SLOT)) GZ_PARTITION=sim$SLOT FASTRTPS_DEFAULT_PROFILES_FILE=~/slot$SLOT/Dronehacks/sim/dds/fastdds_udp.xml && source /opt/ros/humble/setup.bash && $*" 2>&1 |
  grep -v "host keys"

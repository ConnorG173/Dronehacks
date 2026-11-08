#!/usr/bin/env bash
# Copy team packages onto drone 12 and build them in the organizers' stack container.
#
#   bash deploy/deploy_to_drone.sh [ssh-target] -- <dir> [<dir> ...]
#
#   ssh-target  default skybeta-usb (USB-C, 192.168.55.1). Over Wi-Fi: dronehacks@<drone-wifi-ip>.
#               ALWAYS our drone only (skybeta-12): the script refuses any other hostname (DQ rule).
#   <dir>       a ROS package directory or any folder to ship (e.g. a worktree's src/team_loc, planner/).
#               Each lands at ~/team_ws/src/dronehacks/<basename> on the drone.
#
# Example (from the repo root, packages straight from each workstream's worktree, no merge needed):
#   bash deploy/deploy_to_drone.sh skybeta-usb -- \
#       ../Dronehacks-path-following/src/team_path_following ../Dronehacks-path-following/planner \
#       ../Dronehacks-localization/src/team_loc ../Dronehacks-localization/src/team_loc_matcher
#
# Builds with the stack's own team_build.sh (overlay on /root/skybeta_ws, files chowned back).
# Does not touch ~/skybeta, Tailscale, PX4 or drone.env, and never arms anything.
set -euo pipefail
TARGET="skybeta-usb"
if [[ $# -gt 0 && "$1" != "--" ]]; then TARGET="$1"; shift; fi
[[ "${1:-}" == "--" ]] && shift
[[ $# -gt 0 ]] || { sed -n 2,18p "$0"; exit 1; }

host="$(ssh -o ConnectTimeout=8 "$TARGET" hostname)"
[[ "$host" == "skybeta-12" ]] || { echo "REFUSING: $TARGET is '$host', not skybeta-12 (never touch another team's drone)"; exit 1; }
echo "== target $TARGET ($host)"

ssh "$TARGET" 'mkdir -p ~/team_ws/src/dronehacks'
for d in "$@"; do
  [[ -d "$d" ]] || { echo "missing $d"; exit 1; }
  name="$(basename "$(cd "$d" && pwd)")"
  echo "== copy $d -> ~/team_ws/src/dronehacks/$name"
  rsync -az --delete --exclude build --exclude install --exclude log --exclude __pycache__ \
        --exclude out --exclude '*.bag' --exclude runs "$d/" "$TARGET:team_ws/src/dronehacks/$name/"
done

echo "== build in skybeta-stack (team_build.sh)"
ssh "$TARGET" 'docker exec skybeta-stack bash -lc "source /usr/local/bin/skybeta_env.sh 2>/dev/null; team_build.sh 2>&1 | tail -25"'
echo "== installed packages"
ssh "$TARGET" 'ls ~/team_ws/install 2>/dev/null | grep -v -E "^(COLCON|setup|local_setup|_local)" || echo "(none)"'
echo "Done. Next: docs/NET_TEST_PLAN.md"

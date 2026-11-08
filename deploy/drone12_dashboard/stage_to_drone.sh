#!/usr/bin/env bash
# From the laptop: copy this checkout + kit to drone 12 at ~/team_ws/team_staging/conservative_v1 (inert staging:
# nothing is built, launched or changed elsewhere; team_staging carries COLCON_IGNORE so team_build.sh skips it).
#   bash deploy/drone12_dashboard/stage_to_drone.sh [ssh-target]      (default skybeta)
set -euo pipefail
TARGET="${1:-skybeta}"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
host="$(ssh -o ConnectTimeout=8 "$TARGET" hostname)"
[[ "$host" == "skybeta-12" ]] || { echo "REFUSING: $TARGET is '$host', not skybeta-12"; exit 1; }
DEST=team_ws/team_staging/conservative_v1
ssh "$TARGET" "mkdir -p ~/$DEST/repo ~/$DEST/kit && touch ~/team_ws/team_staging/COLCON_IGNORE"
rsync -az --delete --exclude .git --exclude build --exclude install --exclude log --exclude __pycache__ \
  --exclude runs --exclude 'planner/build' --exclude 'planner/out' --exclude '*.bag' --exclude recordings \
  "$ROOT/" "$TARGET:$DEST/repo/"
rsync -az --delete "$ROOT/deploy/drone12_dashboard/" "$TARGET:$DEST/kit/"
echo "$(git -C "$ROOT" rev-parse --short HEAD) $(date -u +%FT%TZ)" | ssh "$TARGET" "cat > ~/$DEST/STAGED_FROM"
echo "staged $(git -C "$ROOT" rev-parse --short HEAD) to $TARGET:~/$DEST  (next, on the drone: bash ~/$DEST/kit/build.sh)"

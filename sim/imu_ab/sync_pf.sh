#!/usr/bin/env bash
# From the laptop: copy the path follower (path_following/ + planner/) at a fixed commit of branch
# path-following into slot 1's working copy, for the IMU-fusion A/B. Run AFTER sim/gcp/sync.sh 1
# (that one copies this branch's planner/, which lacks course_v3_gateb.yaml).
#   bash sim/imu_ab/sync_pf.sh [commit]      default 45bfe51 (path-following tip, 2026-11-08)
set -eo pipefail
REV="${1:-45bfe51}"
source "$(cd "$(dirname "$0")/../gcp" && pwd)/vm.env"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
git -C "$ROOT" archive --format=tar "$REV" path_following planner | gzip |
  gcloud compute ssh "$VM" --project "$PROJECT" --account "$ACCOUNT" --zone "$ZONE" --quiet \
    --command "cd ~/slot1/Dronehacks && tar xzf - && echo path_following+planner @ $REV synced to slot 1" 2>&1 |
  grep -v "host keys"

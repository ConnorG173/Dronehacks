#!/usr/bin/env bash
# From your laptop: copy this checkout to the VM as slot N's working copy (~/slot<N>/Dronehacks).
#   bash sim/gcp/sync.sh 0        (slot 0 = path following, slot 1 = localization)
# Copies files only (no .git, no build output); safe to run after every edit.
set -eo pipefail
SLOT="${1:?usage: sync.sh <slot 0|1>}"
source "$(cd "$(dirname "$0")" && pwd)/vm.env"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
tar czf - --no-xattrs -C "$ROOT" --exclude=.git --exclude=./build --exclude=./install --exclude=./log \
    --exclude=__pycache__ --exclude=sim/gz/models/dronehacks_course --exclude=planner/build --exclude=planner/out . |
  gcloud compute ssh "$VM" --project "$PROJECT" --account "$ACCOUNT" --zone "$ZONE" --quiet \
    --command "mkdir -p ~/slot$SLOT/Dronehacks && tar xzf - -C ~/slot$SLOT/Dronehacks && echo synced to slot $SLOT" 2>&1 |
  grep -v "host keys"

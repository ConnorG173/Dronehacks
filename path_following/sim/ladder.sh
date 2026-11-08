#!/usr/bin/env bash
# One speed-ladder step in slot 0: fly the same settings N times (fresh world each), print one line per run.
#   bash path_following/sim/ladder.sh <step-name> <runs> [launch args...]
#   e.g. bash path_following/sim/ladder.sh la 2 time_scale:=1.0 tracker_params:=/root/team/.../tracker_sim_tune.yaml
# Runs are named <step-name>1, <step-name>2, ... ; full output in /tmp/<run>.out on the VM.
set -o pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
step="${1:?step name}"; n="${2:?runs}"; shift 2
for k in $(seq 1 "$n"); do
  run="${step}${k}"
  bash "$REPO/path_following/sim/slot0.sh" reset >/tmp/${run}.reset 2>&1
  bash "$REPO/path_following/sim/slot0.sh" fly vio "$run" "$@" >/tmp/${run}.out 2>&1
  m="$REPO/path_following/runs/sim/$run/metrics.txt"
  score=$(head -1 "$m" 2>/dev/null)
  vio=$(grep -m1 "at-go (tracker view)" "$m" 2>/dev/null | sed 's/  */ /g')
  lag=$(grep -m1 "^xy: tau_v" "$m" 2>/dev/null)
  echo "== $run | $score"
  echo "   $vio | $lag"
  echo "   $(cat "$REPO/path_following/runs/sim/$run/rtf_summary.txt" 2>/dev/null)"
  grep -E "pilot go failed|POSE FAULT|no plan after|WARNING: still no ground truth" /tmp/${run}.out /tmp/${run}.reset | head -3
done

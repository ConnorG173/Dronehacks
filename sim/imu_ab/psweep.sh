#!/usr/bin/env bash
# Parallel replay sweep (VM): bash sim/imu_ab/psweep.sh <input bag> <tag> <variants file> [workers=4]
# variants file: one "name launch-args..." per line. START_OFFSET / PLAY_SECS (env) trim the replay.
# Prints one summary line per variant: horizontal ATE rms / end, yaw drift at end, tilt max.
set -o pipefail
cd "$(dirname "$0")/../.."
src="${1:?}"; tag="${2:?}"; vf="${3:?}"; W="${4:-4}"
run_one() {
  local rid="$1" name="$2"; shift 2
  REPLAY_ID=$rid bash sim/imu_ab/replay.sh "$src" "${tag}_$name" "$@" > /dev/null 2>&1
  local j="runs_imu/replay/${tag}_$name/vio_drift.json"
  if [ -f "$j" ]; then python3 -c "import json,sys; d=json.load(open('$j')); print(f\"$name ATE rms {d['ate_h_rms']:.3f} end {d['end_h']:.3f} | yaw end {d['yaw_end_deg']:+.1f} max {d['yaw_max_deg']:.1f} | tilt max {d['tilt_max_deg']:.1f} | $*\")"
  else echo "$name FAILED | $*"; fi
}
i=0
while read -r name args; do
  [ -z "$name" ] && continue
  run_one $((i % W + 1)) "$name" $args &
  i=$((i + 1))
  if (( i % W == 0 )); then wait; fi
done < "$vf"
wait
echo PSWEEP_DONE

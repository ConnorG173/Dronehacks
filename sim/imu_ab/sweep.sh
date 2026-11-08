#!/usr/bin/env bash
# Replay sweep on one recorded input (VM, ~/slot1/Dronehacks):  bash sim/imu_ab/sweep.sh runs_imu/<run>/input <tag>
# One replay per line of VARIANTS: name, then launch args. Summary table at the end.
set -o pipefail
cd "$(dirname "$0")/../.."
src="${1:?input bag}"; tag="${2:?tag}"
VARIANTS="${VARIANTS:-off enable_imu_fusion:=False
on enable_imu_fusion:=True
on200 enable_imu_fusion:=True imu_frequency:=200.0
onx10 enable_imu_fusion:=True gyro_noise_density:=0.00244 accel_noise_density:=0.01862 gyro_random_walk:=0.0002 accel_random_walk:=0.03
onyaw90 enable_imu_fusion:=True imu_yaw:=1.5708}"
while read -r name args; do
  [ -z "$name" ] && continue
  echo "=== $(date -u +%T) ${tag}_$name: $args"
  bash sim/imu_ab/replay.sh "$src" "${tag}_$name" $args 2>&1 | grep -E "Enable IMU|frequency|vio_drift|horiz ATE|attitude drift" | sed 's/^/   /'
done <<< "$VARIANTS"
echo SWEEP_DONE

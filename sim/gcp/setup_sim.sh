#!/usr/bin/env bash
# Stage 2, run from a checkout of this repo on the VM (after setup_vm.sh):
#   bash sim/gcp/setup_sim.sh
# Generates the Gazebo course model from the STEP, installs the skybeta airframe into PX4 and rebuilds SITL.
# Idempotent.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
step() { echo; echo "=== [$(date +%H:%M:%S)] $*"; }

step "python deps (cadquery for STEP, trimesh, pillow, pymavlink)"
python3 -m pip install --user -q cadquery trimesh pillow numpy pymavlink

step "course model from dronehacks-course-v3.step"
python3 "$REPO/sim/tools/step_to_gz.py" "$REPO/dronehacks-course-v3.step" --out "$REPO/sim/gz/models/dronehacks_course"

step "skybeta airframe -> PX4"
AF="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
cp "$REPO/sim/px4/4099_gz_skybeta" "$AF/4099_gz_skybeta"
grep -q "4099_gz_skybeta" "$AF/CMakeLists.txt" || sed -i 's/^\(\s*\)4001_gz_x500$/&\n\14099_gz_skybeta/' "$AF/CMakeLists.txt"
grep -n "4099_gz_skybeta" "$AF/CMakeLists.txt"

step "rebuild PX4 SITL"
cd "$PX4_DIR"
export PATH="$HOME/.local/bin:$PATH"
make px4_sitl -j"$(nproc)"

step "DONE. Start the sim with: bash sim/run_sim.sh"

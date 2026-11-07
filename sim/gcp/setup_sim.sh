#!/usr/bin/env bash
# Stage 2, run from a checkout of this repo on the VM (after setup_vm.sh):
#   bash sim/gcp/setup_sim.sh
# Generates the Gazebo course model from the STEP, installs the skybeta airframe into PX4 and rebuilds SITL.
# Idempotent.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
step() { echo; echo "=== [$(date +%H:%M:%S)] $*"; }

step "python deps"
# cadquery/trimesh live in their own venv: they pull NumPy 2, which breaks the system SciPy/ROS
# Python (NumPy 1.x ABI) if installed into ~/.local. pymavlink (no NumPy) goes in the user site.
python3 -m pip uninstall -y -q numpy 2>/dev/null || true   # undo an earlier --user NumPy 2, if any
python3 -m pip install --user -q pymavlink
sudo apt-get install -yq python3-venv >/dev/null
VENV="$HOME/.venvs/simtools"
[[ -x "$VENV/bin/python" ]] || python3 -m venv "$VENV"
"$VENV/bin/pip" install -q --upgrade pip
"$VENV/bin/pip" install -q cadquery trimesh pillow numpy

step "course model from dronehacks-course-v3.step"
"$VENV/bin/python" "$REPO/sim/tools/step_to_gz.py" "$REPO/dronehacks-course-v3.step" --out "$REPO/sim/gz/models/dronehacks_course"

step "skybeta airframe -> PX4"
AF="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
cp "$REPO/sim/px4/4099_gz_skybeta" "$AF/4099_gz_skybeta"
grep -q "4099_gz_skybeta" "$AF/CMakeLists.txt" || sed -i 's/^\(\s*\)4001_gz_x500$/&\n\14099_gz_skybeta/' "$AF/CMakeLists.txt"
grep -n "4099_gz_skybeta" "$AF/CMakeLists.txt"

step "PX4 rcS: allow un-namespaced uXRCE-DDS for instance > 0 (PX4_NO_UXRCE_NS=1)"
RCS="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/rcS"
grep -q PX4_NO_UXRCE_NS "$RCS" || sed -i 's/^\(\s*\)uxrce_dds_ns="-n px4_$px4_instance"/\1[ -z "$PX4_NO_UXRCE_NS" ] \&\& uxrce_dds_ns="-n px4_$px4_instance"/' "$RCS"
grep -n 'uxrce_dds_ns=' "$RCS"
# Same flag also keeps MAV_SYS_ID = 1 for every instance: the organizers' flight interface addresses PX4
# as system 1 (as on the drone), so instance N's default sysid N+1 would ignore its Offboard request.
grep -q 'PX4_NO_UXRCE_NS" ] && param set MAV_SYS_ID' "$RCS" || \
  sed -i 's/^\(\s*\)param set MAV_SYS_ID \$((px4_instance+1))/\1if [ -n "$PX4_NO_UXRCE_NS" ]; then param set MAV_SYS_ID 1; else param set MAV_SYS_ID $((px4_instance+1)); fi/' "$RCS"
grep -n 'MAV_SYS_ID' "$RCS"

step "rebuild PX4 SITL"
cd "$PX4_DIR"
export PATH="$HOME/.local/bin:$PATH"
make px4_sitl -j"$(nproc)"

step "DONE. Start the sim with: bash sim/run_sim.sh"

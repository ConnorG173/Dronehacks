# takeoff_land

This is a complete editable companion flight project. New lab workspaces receive
these ordinary Python files and their configuration automatically. Copy the full
project and declare its dependencies when preparing hardware deployment.

Open **FLIGHT_CODE.md** for the source map. `main.py` starts the local `refnav/`
package: edit `refnav/core.py` for the climb, hover, navigation and landing
decisions, `refnav/node.py` for the ROS 2/PX4 interface, and `refnav/depth.py` for
depth processing. Run `python3 -m pytest -q tests` for the local decision tests.
Run **main.py** after editing helpers; reset after a previous run:

```sh
python3 main.py
```

Plain Python directly starts the local ROS node, like on a real companion; it
does not submit a lab operation. The optional workbench Run button
snapshot your project and provide admission, recording and independent grading.
Direct terminal programs need their own recorder, such as `ros2 bag record`.
Use Reset simulation in the Session UI when you need a fresh simulated vehicle.
The cloud companion also provides `python` as Python 3 when no interpreter of
that name has already been installed. Outside the cloud, use the reviewed
local/hardware deployment configuration. This source imports no runtime library:
`flight.yaml`, `route.json` and `refnav/launch.py` are ordinary editable files.
`/etc/flyx/mission.yaml` (or `DRONE_PARAMS_FILE`) may supply the deployment's ROS
parameters. The hardware default clock is system time; simulation deployment
selects simulated time. Both use the native topic/frame contract documented in
`docs/NATIVE_DRONE_INTERFACE.md` in the toolkit. Physical launch is not qualified.

The initial algorithm is copied unchanged from the maintained reference package.
Its earlier baseline flew on the rig on 2026-10-18 (docs/CURRENT_STATE.md); the
workspace packaging change is separate from moving-flight qualification.

`onboard/` contains the actual editable cuVSLAM, vision bridge, health/odometry
nodes, launch configuration and process manager. See `onboard/README.md`.
`python3 onboard/services.py status` names the running source and log files;
stop/start explicitly after editing perception code. Startup never arms.

---

# Team project: planner + path follower

*(Everything above this line is the lab's text and is left unchanged. This section is ours.)*

```
planner/            C++ GCOPTER + OMPL planner: course map + checkpoints -> path.bin (no ROS)
  src/ CMakeLists.txt config/course_v3.yaml maps/course_v3.bin tests/ third_party/gcopter/ tools/
localizer/          ROS 2 ament_python package "localizer": flies path.bin
  localizer/core/   the follower: plain Python + numpy, NO ROS (path, frames, PathClock, fence,
                    state machine). Both adapters run exactly this code.
  localizer/sim.py  simulator adapter: px4_msgs, /fmu/in/* direct, full feedforward, arms itself
                    only with --i-have-team-approval, lands with NAV_LAND
  localizer/real.py real-drone adapter (skybeta): publishes only /skybeta/setpoint/pose (the lead
                    point), preview topic by default, never arms
  launch/ data/course_v3_path.bin test/ HALF_ASSED.md (known shortcuts)
dronehacks-course-v3.step   course CAD (source of planner/maps/course_v3.bin)
```

The planner turns the course CAD and an ordered list of checkpoints into one time-sampled,
validated trajectory: `path.bin`, PX4 local NED at the launch pad, sampled every 0.01 s (format
in `planner/README.md`). The follower core loads it into the course frame and computes, every
50 ms, where the vehicle should be. Its inputs are the vehicle's pose and its armed/Offboard/landed
state; its outputs are p, v, a, yaw and an on-path lead point q. It runs the run's state machine:
pad init, takeoff, follow with a time-governed path clock, hold on fault, and a land request at the
end. The adapters only translate ROS messages to and from those dataclasses. Where the simulator and
the drone differ (QoS, clock, feedforward vs lead point, who arms, how a run ends), the difference
is an adapter choice or a `Config` value, never a branch in the core.

## Run in the simulator

Preconditions, every time: **the team has approved this mission**. Keyboard RC landed and released,
then **Reset world** in the Session UI. Nothing else is flying (no `main.py` or notebook mission). The
VIO services are running (`python3 onboard/services.py status`; PX4 refuses position-controlled
arming without them). The spawn pose matches the course start (-2.51, 0.91), heading +x: check it
with `python3 SIM_SENSOR_EXAMPLE.py`. The follower assumes it starts on the launch mark.

In the companion terminal (Workbench, Ctrl+`), from the workspace `/var/lib/flyx-lab/workspace`:

```sh
cd localizer
python3 -m localizer.sim                           # passive: creates no publisher, only logs
python3 -m localizer.sim --i-have-team-approval    # arms, takes off, flies the path, lands
```

With the flag it also refuses to start unless `FLYX_LAB_ACTIVE=1` and `FLYX_HITL=0` and sim time
is on (`echo $FLYX_LAB_ACTIVE $FLYX_HITL`). It exits 0 after "landed and disarmed", 1 on any abort;
it yields (stops streaming) if Offboard is lost, holds and then lands on a fault. Optional ROS parameters go after `--ros-args`, e.g. `-p max_path_speed_mps:=0.5`.
With a colcon-built workspace: `ros2 launch localizer sim.launch.py approved:=true`.
A terminal run is not a graded Run; record it yourself with `ros2 bag record` if needed.

## Run on the drone (skybeta Jetson, ROS 2 Humble)

Each step needs the team's approval. The pilot arms on RC and switches to Offboard; our code never
arms and never touches `/fmu/in`.

1. **Deploy** only the package to the team workspace (`~/team_ws/src/localizer`; never `~/connor`),
   for example from the laptop repo root (Git Bash; excludes bytecode and tests):
   `tar --exclude=__pycache__ -cf - -C localizer localizer launch data resource package.xml setup.py setup.cfg | ssh skybeta 'mkdir -p ~/team_ws/src/localizer && tar -xf - -C ~/team_ws/src/localizer'`
2. **Build** in the `skybeta-stack` container with the team build script (`team_build.sh`), as for
   any team package.
3. **Preview** (props off is fine), next to the stack's running flight_interface:
   `ros2 launch localizer real.launch.py`
   then watch `/localizer/status` (mode, T, `expected_trajectory_setpoint_ned`, `expected_yaw_ned_deg`
   vs `px4_yaw_ned_deg`) and `/localizer/setpoint_preview`. `dry_run:=true` plays the whole path
   on the preview topic without a vehicle.
4. **Bench, props off**: `ros2 launch localizer real.launch.py live:=true`
   and check `/fmu/in/trajectory_setpoint`; measure the yaw offset (HALF_ASSED.md).
   Pass `mpc_xy_p:=<the drone's MPC_XY_P>` and `command_yaw:=false` until the yaw agrees.
5. **Flight**: tethered hover, then pad to M1 at <= 1 m/s, then the full course with a speed ramp
   (`max_path_speed_mps:=`). To start it from the stack instead, set `COMPETITOR_PKG=localizer` and
   `COMPETITOR_LAUNCH=real.launch.py` in `/etc/skybeta/drone.env` (ask first); there it needs
   `with_flight_interface:=true` (default false, so a manual launch never starts a second one) and
   `live:=true` to fly. How the stack passes launch arguments is not verified.

## Run the tests (laptop, no ROS)

```sh
py -m pytest -q localizer          # core, state machine, adapter conversions, offline closed loop
py -m pytest -q tests              # the lab's refnav tests
py localizer/test/closed_loop.py --png closed_loop.png   # sim vs real adapter on the PX4 model
```

Planner (Windows: MSYS2 UCRT64 + OMPL 1.7.0, see `planner/README.md`; Linux: apt packages):

```sh
cd planner
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="C:/Users/Conno/projects/ompl-1.7.0;C:/msys64/ucrt64"
cmake --build build -j
py tests/run_tests.py build        # parity, checkpoints, file format; some checks are timing/RRT-flaky
./build/gcopter_planner config/course_v3.yaml    # a new plan (random: differs every run)
```

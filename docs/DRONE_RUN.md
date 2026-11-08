# Drone run: team_path_following on skybeta-12 (tonight)

Branch `path-following`. Claude sessions are read-only on the drone and never arm or fly it.
The safety pilot is in charge the whole time. Background: `path_following/DRONE_BRINGUP.md`.

## 1. What goes on the drone

**Clone the repo whole** to `~/team_ws/src/Dronehacks` (in containers: `/root/team_ws/src/Dronehacks`).
At runtime the node needs the planner binary, `planner/config/*.yaml` and `planner/maps/course_v3.bin`
from the same checkout.

| Piece | Path in repo | Build |
|---|---|---|
| ROS package `team_path_following` (ament_python, node `mission_tracker`, `launch/mission.launch.py`) | `path_following/team_path_following` | `team_build.sh --packages-select team_path_following` |
| GCOPTER planner (plain CMake, **not** a ROS package; colcon ignores it) | `planner/` | `cmake -S planner -B planner/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/ros/humble && cmake --build planner/build -j4 --target gcopter_planner` |
| course config + map | `planner/config/course_v3_gateb.yaml`, `planner/maps/course_v3.bin` | none |

Build in the stack container (`~/skybeta/host/run.sh shell`, then `cd /root/team_ws/src/Dronehacks`).
Check the build: `ls planner/build/gcopter_planner`, and `ros2 pkg executables team_path_following` should
print `mission_tracker`.

**Dependencies: nothing to add.** I checked the running `skybeta:0.3.2` stack container on skybeta-12,
read-only, 16:35 PST:

- `ros-humble-ompl` 1.7.0. `Dockerfile.jetson` doesn't name it; it arrives with ROS:
  `ros-humble-moveit-planners-ompl` is installed too.
- `libeigen3-dev` 3.4.0, `cmake` 3.22, `g++` 11.
- Python: `numpy` 1.26.4. Everything else the node uses is the standard library, rclpy and the message
  packages already in the image.

The other `~/team_ws/src` package now is `localizer` (localization's). It's needed only if you fly on
`/team/odom_map` (section 2, variant B).

## 2. Launch (stack container, one command)

`mission.launch.py` starts the organizers' `flight_interface` with the same arguments and defaults as
`competitor_example/competitor.launch.py` (`max_speed_mps` 1.5, `setpoint_rate_hz` 10,
`allow_mode_request` false, `auto_offboard_on_arm` true, `odometry_topic` /visual_slam/tracking/odometry,
altitude band 0.3–3.0), plus `mission_tracker`.

**Before launching, make sure no other `flight_interface` is running** (`ros2 node list | grep
flight_interface`). Two interfaces fight. On skybeta-12 the competitor window currently runs
`team_ws/claude/startup/run_competitor.sh` (a dashboard launch); check that it doesn't start one.

**Variant A, tonight's default: raw VIO + launch-pose fallback** (no localization package needed):

```bash
R=/root/team_ws/src/Dronehacks
ros2 launch team_path_following mission.launch.py \
  repo_root:=$R course_config:=$R/planner/config/course_v3_gateb.yaml \
  planner_binary:=$R/planner/build/gcopter_planner out_dir:=/root/team_ws/bags/pf_$(date +%H%M) \
  pose_topic:=/visual_slam/tracking/odometry pose_topic_frame:=odom \
  base_height_on_pad_m:=<measured, see below> \
  setpoint_rate_hz:=10.0 max_speed_mps:=1.5 auto_offboard_on_arm:=true allow_mode_request:=false \
  mission_preset:=<hover | first_leg | course>  [time_scale:=1.4 for course]
```

- **Variant B: on localization.** Run team_loc first (their command, in `localization-integration.md`:
  `ros2 launch team_loc team_loc.launch.py course_config:=$R/planner/config/course_v3_gateb.yaml
  base_height_on_pad_m:=<same>`). Then use the command above **without** the two `pose_topic*` arguments;
  the defaults are `/team/odom_map`, frame `course`. You get localization's ERROR/WARN status for free:
  ERROR holds, WARN slows.
- **Course config on the drone:** `/root/team_ws/src/Dronehacks/planner/config/course_v3_gateb.yaml`
  (Gate B route). The window route is `course_v3.yaml`, not for tonight.
- **`base_height_on_pad_m`** is base_link (the Pixhawk IMU) height above the floor with the drone on the
  pad. Not measured yet. Measure with a ruler: floor to the centre of the OAK-D lens, then subtract
  `CAMERA_Z` 0.0304 (`/etc/skybeta/drone.env`). Expect 0.05–0.12 m. Leaving it at 0 makes the drone
  fly that much higher than planned; tolerable, but measure it.
- **Placement matters in variant A.** The course frame comes from the pose at arming, assumed to be the
  config's `StartPos` (launch mark centre) facing `StartHeadingDeg` 0, i.e. map +x, east, into the maze.
  Centre the drone on the launch mark, nose toward +x. A 10° yaw error is about 0.5 m of error at the far end.
- **Planning takes about 15–60 s at startup.** It's GCOPTER at 5 cm voxels, fastest of 3 candidates,
  within a 45 s budget. Wait for `plan ready: … s` in the log. Before that the node publishes nothing, so
  arming gives no autonomy. `NO PLAN` means it will never fly.
- **Record a bag of every flight:** `~/skybeta/host/run.sh bag` plus `/team/tracker/debug
  /team/mission/state /team/plan/path`. Afterwards, `python3 path_following/tools/fit_lag.py <bag>
  --states course` gives the real airframe's lag.

## 3. Presets (`mission_preset:=`)

All three end with a landing and PX4's auto-disarm. Tested end to end through ROS
(`tools/e2e_docker.sh loc`, kinsim stand-in for interface + PX4 + localization).

| preset | does | duration (e2e) | args |
|---|---|---|---|
| `hover` | takes off over the launch mark to `HoverHeight` (1.0 m), holds 10 s, lands back on the launch mark | takeoff 2.6 s + 10 s + land 2 s | `hover_hold_s:=10.0` |
| `first_leg` | takes off, flies slowly (planner `MaxVelMag` 0.4 m/s) to checkpoint 1 (M1 maze gap, 2.1 m away), holds 5 s, **lands there** | 13 s plan (8 s travel + 5 s hold) | `first_leg_speed_mps:=0.4 first_leg_hold_s:=5.0` |
| `course` + `time_scale:=1.4` | full Gate B route (maze M1–M3 → tube → chicane S through Gate B → pad), plan slowed 1.4× (peak 0.86 m/s), lands on the pad | 27.3 s course | `time_scale:=1.4` |

- **first_leg lands in the M1 gap**, between the divider end and the north net (about 0.46 m to each).
  The Sky Beta reaches 0.29 m, so there's about 17 cm of margin while descending. The pilot should be
  ready to take over if it drifts.
- **Why time_scale 1.4 for the course.** In kinsim on the Gate B plan, across four plant models
  (nominal, Gazebo-matched lag, slower/laggier airframe, noisy VIO), it gave 0 wall touches and
  ≥ 0.20 m clearance. 1.0 also had 0 touches, but at 1.4 the cross-track error on the laggy plant drops
  from 0.29 to 0.17 m. Go to 1.2, then 1.0, only after clean flights; check `e_cross` in
  `/team/tracker/debug` (< 0.15 m).
- **Gazebo:** Gate B runs on the new textures are in progress (vio20/21). See
  `docs/results/path_following.md`.

## 4. Pilot procedure, abort, what the pilot sees

1. Stack running, drone still on the launch mark (nose +x), our launch started, `plan ready` in the log.
   `/team/mission/state` = `idle`. `/skybeta/setpoint/twist` streams zeros at about 50 Hz.
2. **Arm in Altitude, then switch to Position within 10 s.** PX4 disarms 10 s after arming without a
   takeoff. The interface switches to Offboard because our setpoints are fresh; the log shows
   `offboard_active: true`. **This differs from the organizers' example:** our node takes off by itself
   the moment Offboard is active and armed. It doesn't wait for a separate command.
3. What the pilot sees:
   - **Takeoff:** straight up at up to 1 m/s; PX4's 3 s takeoff ramp makes the first second slow.
   - **Course:** starts within 0.15 m of 1.0 m height. `hover` holds still; `first_leg` and `course`
     move at about 0.4 m/s or up to 0.86 m/s.
   - **Landing:** over the end point at 0.8 m/s, slowing to 0.3 m/s below 0.25 m, then pressing down
     until PX4 auto-disarms (about 2 s after touchdown). State: `landing` → `landed` → `done`.
4. **Abort: flip the mode switch out of Position** (Altitude or Manual), or take over per the organizers'
   RC procedure. Offboard ends, the node goes to `abort` and publishes zeros. It **never re-engages** in
   that run; a new run needs a new arm. Fly it down manually.
5. Automatic holds the pilot may see:

   | trigger | what happens |
   |---|---|
   | pose older than 0.25 s | zero velocity: PX4 brakes and holds |
   | localization ERROR (variant B) | hold, path clock stops |
   | localization WARN (variant B) | slower |
   | our node dies or stops sending for more than 1 s | the interface holds position |

   The node enforces a z ceiling of 1.6 m (68 in rule), and the interface clamps speed to 1.5 m/s.
6. **Kill switch on the laptop side:** Ctrl-C on the launch. Twists stop, and after 1 s the interface
   holds; then the pilot takes over.
7. **Don't fly over the walls.** PX4's height reference is the downward rangefinder, and it sees the wall
   tops. The planner is capped at z 1.6 m, so plans never do.

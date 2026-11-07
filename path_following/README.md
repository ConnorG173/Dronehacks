# path_following

Plan once, then fly the plan with **velocity setpoints only** through the organizers' flight interface.
Design and interface findings: `docs/proposals/path_following.md`. Measurements: `docs/results/path_following.md`.

```
team_path_following/          ROS 2 Humble package (ament_python)
  team_path_following/
    node.py                   mission_tracker: plan -> track -> /skybeta/setpoint/twist
    mission.py                idle -> takeoff -> course -> landing -> landed -> done | abort
    tracker.py                inverse-lag feedforward + cross-track-priority feedback + time-scaled clock
    pathfile.py frames.py     DHPATH loading, planner NED -> map ENU -> odom
    course.py clearance.py    planner config / DHMAP map, distance field
    planner_runner.py         runs planner/ (unchanged) with overrides; keeps the fastest of N plans
  launch/mission.launch.py    flight_interface (exactly as competitor.launch.py) + mission_tracker
  config/tracker.yaml         gains
tools/                        offline: kinsim.py, sweep.py, fit_lag.py, make_random_map.py, plan.py,
                              kinsim_ros.py + e2e_docker.sh (ROS end-to-end), eval_bag.py
sim/slot0.sh                  Gazebo slot 0: build / fly gt|vio / stop
```

## Safety behaviour (read before flying)

- **Never sends a pose.** A pose becomes the flight interface's hold point, and a lapsed twist stream
  would fly the vehicle back to it at up to 12 m/s.
- Publishes **nothing until a validated plan exists**, so arming without a plan gives no autonomy.
  With a plan, it streams zero twists so the pilot's arm → Position switch can enter Offboard.
- Stale pose (> 0.25 s) or localization ERROR: zero velocity. Localization WARN: half speed.
  Offboard lost: `abort`, never tries to re-enter.
- Enforces its own altitude ceiling (`z_ceiling_m`, map z, default 1.6 m): the interface does not
  altitude-clamp twists.

## Run

Offline (laptop, Docker Desktop running):

```bash
docker build -t pf-dev:humble path_following/docker                       # once
python3 path_following/tools/plan.py planner/config/course_v3.yaml path_following/runs/c MaxVelMag=1.2
python3 path_following/tools/kinsim.py planner/config/course_v3.yaml path_following/runs/c/path.bin --plot /tmp/k.png
python3 -m pytest -q path_following/team_path_following/test
bash path_following/tools/e2e_docker.sh                                     # real node + launch file, ROS 2
```

Gazebo slot 0 (`docs/SIM.md`):

```bash
bash sim/gcp/sync.sh 0
bash sim/gcp/vm.sh 0 'bash path_following/sim/slot0.sh build'
bash sim/gcp/vm.sh 0 'bash path_following/sim/slot0.sh fly gt  run01'       # ground-truth pose: tracker only
bash sim/gcp/vm.sh 0 'bash path_following/sim/slot0.sh fly vio run02'       # cuVSLAM: the drone's fallback
```

Each run appends a row to `docs/results/path_following.md` and prints the fitted PX4 lag. Put
`tau_v`/`t_delay` into `config/tracker.yaml`.

Drone (after the sim works; team decision): build the planner in the stack container
(`cmake -S planner -B planner/build -DCMAKE_PREFIX_PATH=/opt/ros/humble && cmake --build planner/build`), `team_build.sh`, then set
`COMPETITOR_PKG=team_path_following` and `COMPETITOR_LAUNCH=mission.launch.py` in `/etc/skybeta/drone.env`.
Launch arguments: `speed_factor` (path-clock rate, 1.0 = as planned), `planner_overrides`
(`"MaxVelMag=1.2"`), `max_speed_mps` (must match the interface; raising it needs the organizers' OK),
`setpoint_rate_hz`, `pose_topic` (default `/team/odom_map`), `base_height_on_pad_m`,
`return_config` (`''` off, `reverse`, or a config path: touch-and-go return leg, only with the organizers' OK).

## Libraries

numpy, scipy (in the skybeta image); the GCOPTER planner in `planner/` (Eigen, OMPL).
`tools/stub_fi/` is an offline test stub with `COLCON_IGNORE`. It must never be built on the drone.

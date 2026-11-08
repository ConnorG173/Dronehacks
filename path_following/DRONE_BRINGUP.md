# Drone bring-up: team_path_following on skybeta-12

For the humans flying tonight. Claude sessions are read-only on the drone and never arm or fly it.
Every step keeps the safety pilot in charge (RC takeover: raise the throttle, or flip the mode switch).

## 0. What this flies

`mission.launch.py` = the organizers' `flight_interface` (unchanged, exactly as in `competitor.launch.py`) +
`mission_tracker`. It plans the course at startup (GCOPTER, ~10–30 s), publishes **nothing** until a plan
exists, then zero twists. After the pilot arms (Altitude) and switches to Position on the pad, it takes off,
flies the plan with velocity setpoints, lands on the pad, and PX4 auto-disarms. **It never sends a pose.**
`/team/mission/state`: idle → takeoff → course → landing → landed → done (abort on any Offboard loss).

**Route (default): Gate B**, `planner/config/course_v3_gateb.yaml`: maze M1–M3 → tube → chicane S through
Gate B (the opening at the north end of the C2 wall line) → pad. Plan ~19.6 s at MaxVelMag 1.2; z capped at
1.6 m in the planner (68 in rule). The window route (+5) is `course_config:=<repo>/planner/config/course_v3.yaml`.
Gazebo showed VIO jumping at the window: a flat wall fills the stereo view on approach. Fly it only after
Gate B finishes reliably. Tracker lag `tau_v` 0.66 / `t_delay` 0.18 (Gazebo fit). Over-estimating the lag
is benign; under-estimating touched walls in kinsim.

## 1. Build (drone host shell, in the team workspace; the repo is cloned in `~/team_ws/src`)

```bash
~/skybeta/host/run.sh shell                       # shell in the running stack container (nothing is edited in ~/skybeta)
cd /root/team_ws/src/Dronehacks
cmake -S planner -B planner/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/ros/humble
cmake --build planner/build -j4 --target gcopter_planner
team_build.sh --packages-select team_path_following
```

Check: `ls planner/build/gcopter_planner`; `ros2 pkg executables team_path_following` → `mission_tracker`.

## 2. Bench, props OFF

```bash
# stop the competitor window's default launch first (two flight interfaces fight), then:
ros2 launch team_path_following mission.launch.py base_height_on_pad_m:=<measured>   # stack container
```

Expect within ~30 s: `plan ready: … s` in the log; `/team/plan/path` in Foxglove over the course map;
`ros2 topic hz /skybeta/setpoint/twist` ≈ 50 Hz, all zeros; `/team/mission/state` = idle.
Arm (Altitude) → Position: `/skybeta/state` `offboard_active: true`, mission → `takeoff`, twist z = +1.0.
Disarm with the switch. Mission → `abort` (correct: Offboard ended in flight).

With the drone carried by hand (props off, armed is NOT needed): watch `/team/tracker/debug` p_x/p_y
follow the carried drone in the course frame (field order in the message's layout label).

## 3. Permanent launch (organizer-sanctioned way)

`/etc/skybeta/drone.env`: `COMPETITOR_PKG=team_path_following`, `COMPETITOR_LAUNCH=mission.launch.py`,
restart the stack on the pad (keep it still: VIO origin re-zeroes). Default launch args are the race
config; to change one, edit the launch defaults or pass args via the competitor window by hand.

## 4. First flights (props on, low, tethered, pilot ready)

1. **Hover + land only**: `planner_overrides:="MaxVelMag=0.5"` and `speed_factor:=0.5`.
   Watch: takeoff to HoverHeight, course clock starting, slow motion, landing on the pad, auto-disarm.
2. **Lag fit**: record a bag of every flight (`~/skybeta/host/run.sh bag`, add `/team/tracker/debug`).
   `python3 path_following/tools/fit_lag.py <bag> --states course` → put `tracker.tau_v` / `tracker.t_delay`
   in `config/tracker.yaml`. Under-estimating `tau_v` is what causes wall touches; over-estimating is benign.
3. **Speed steps**: `speed_factor` 0.5 → 0.75 → 1.0 (plan MaxVelMag 1.2). Stop at the first flight with
   VIO WARN/ERROR, a clamp warning, or > 0.15 m cross-track (`e_cross` in the debug topic).
4. Optional: `speed_factor:=1.1/1.15` (validated offline only).

## 5. What to watch, and what it means

| Symptom | Meaning / action |
|---|---|
| `NO PLAN, publishing nothing` | planner failed 3×; arming gives no autonomy (safe). Check course config/map paths |
| `auto offboard: staying out of Offboard: …` | interface refused (order: arm in Altitude, THEN Position) |
| mission stuck in `takeoff` | height never reached: check odom z vs `HoverHeight`, `base_height_on_pad_m` |
| `stale pose: zero velocity` | VIO/odom_map older than 0.25 s; the drone brakes |
| `localization ERROR: holding` | team_loc says ERROR; path clock stops |
| `flight interface clamped our setpoint` | `max_speed_mps` lower than our budget, or a bad command |
| drifts sideways at constant command | odom vs PX4 heading residual (guide §1): compare yaws, report |
| height jumps over a wall | rangefinder height (EKF2_HGT_REF=2) sees the wall top; don't fly over walls |

## 6. Rules this respects

Only `/skybeta/setpoint/twist` (never `/fmu/in`), flight interface unchanged, envelope not loosened
(`max_speed_mps` 1.5 unless the organizers allow more), no run data reused (the plan is made from the
published map at every start), the 68 in ceiling enforced in our node (`z_ceiling_m` 1.6).

# Path following: quick guide

Everything lives on branch **`path-following`**, in the ROS 2 package
**`team_path_following`** (`path_following/team_path_following/`). It plans the course once at startup
with the GCOPTER planner (`planner/`), then flies the plan by streaming velocity setpoints to the
organizers' flight interface. More detail:
- Tonight's drone procedure: `docs/DRONE_RUN.md`
- Bring-up and symptoms: `DRONE_BRINGUP.md`
- Design: `docs/proposals/path_following.md`
- Numbers: `docs/results/path_following.md`

## Profiles: which launch file to fly

| launch file | what it is | use for |
|---|---|---|
| **`conservative.launch.py`** | **Frozen**: tonight's validated drone settings. Gate B route, plan slowed 1.4× (peak 0.86 m/s), lag 0.66/0.18 from `config/tracker_conservative.yaml`. Only safe arguments pass through. Git tag `drone-conservative-v1`. | the drone |
| `mission.launch.py` | Everything adjustable: speed, gains file, course, planner overrides | Gazebo and tuning |

Rule: **tune `tracker.yaml`, `tracker_sim_x500.yaml` or launch args, never `tracker_conservative.yaml`.**
When a faster setting has proven itself, add a new profile (e.g. `fast.launch.py`) next to it.

Both profiles take `mission_preset:=hover | first_leg | course`. The default is `hover`, so a stack
restart never flies the course by surprise.

| preset | flight |
|---|---|
| `hover` | take off to 1.0 m over the launch mark, hold 10 s, land |
| `first_leg` | fly slowly (0.4 m/s) to checkpoint 1, hold 5 s, land there |
| `course` | maze → tube → chicane through Gate B → land on the pad |

## Drone (one command, in the stack container)

```bash
R=/root/team_ws/src/Dronehacks
ros2 launch team_path_following conservative.launch.py repo_root:=$R mission_preset:=hover \
  pose_topic:=/visual_slam/tracking/odometry pose_topic_frame:=odom base_height_on_pad_m:=<measured>
```

- To fly on localization instead (`/team/odom_map`, needs team_loc running), drop the two `pose_topic*`
  arguments.
- Before the first flight: build steps, pilot procedure and abort are in `docs/DRONE_RUN.md`.

## Gazebo (GCP VM, slot 0)

```bash
bash sim/gcp/sync.sh 0                                                    # push this checkout to the VM
bash sim/gcp/vm.sh 0 'bash path_following/sim/slot0.sh build'             # after code changes
bash sim/gcp/vm.sh 0 'bash path_following/sim/slot0.sh reset'             # fresh world, waits for check_sim OK
bash sim/gcp/vm.sh 0 'bash path_following/sim/slot0.sh fly vio run30 tracker_params:=/root/team/path_following/team_path_following/config/tracker_sim_x500.yaml'
bash sim/gcp/vm.sh 0 'LAUNCH=conservative.launch.py bash path_following/sim/slot0.sh fly vio run31 mission_preset:=course'
```

- `fly gt` uses ground truth as the pose (but PX4 still fuses VIO). `fly vio` uses raw cuVSLAM, the
  drone's fallback. `fly loc` uses `/team/odom_map`.
- Each run writes `path_following/runs/sim/<name>/` on the VM: the bag, `metrics.txt` and
  `results_row.md`. `metrics.txt` covers:
  - course time
  - cross-track error
  - wall clearance
  - checkpoints
  - PX4 lag fit
  - VIO error vs ground truth
- Reset between runs. Gazebo needs about 2.5 min to load textures.
- **Check the real-time factor.** Every run logs Gazebo's RTF during the flight (`rtf_summary.txt`) and
  flags RTF < 0.9 as invalid for timing. With three sims on the 16-vCPU VM, slot 0 dropped to 0.19×
  real time, and course times and lag fits doubled. In slot 0, `mission_tracker` now runs on sim time
  (`tracker_use_sim_time:=true`, the slot0.sh default, `SIM_TIME=false` to disable), so a slow sim
  stretches wall time instead of faking lag. The flight interface stays on wall time; on the drone,
  everything is wall time.
- Gate B in Gazebo so far: vio20 was clean (0 touches, 6/7 checkpoints, VIO error 2.4 cm RMS). vio21
  failed: cuVSLAM jumped 9.5 m at liftoff. The node now has a pose jump guard (`pose_jump_m`). On a jump
  it aborts: in flight it descends slowly in place, on the pad it never takes off.

## Offline (laptop, seconds)

```bash
python3 path_following/tools/plan.py planner/config/course_v3_gateb.yaml path_following/runs/gb MaxVelMag=1.2
python3 path_following/tools/kinsim.py planner/config/course_v3_gateb.yaml path_following/runs/gb/path.bin \
  --set tau_v=0.66 --set t_delay=0.18 --plot /tmp/k.png --json        # PX4 + interface + VIO model
python3 -m pytest -q path_following/team_path_following/test             # unit tests
LAUNCH_FILE=conservative.launch.py bash path_following/tools/e2e_docker.sh vio mission_preset:=course   # real ROS node
```

`kinsim --set sim.link_delay_s=0.14` reproduces Gazebo's PX4 lag. `--speed-scale 1.4` is the
conservative timing.

## Tools for logs (sim or real bags)

| tool | answers |
|---|---|
| `tools/fit_lag.py BAG --states course` | PX4 velocity lag (tau_v, t_delay) to put in the gains file |
| `tools/eval_bag.py BAG COURSE PATH` | course time, cross-track, clearance, checkpoints (needs ground truth) |
| `tools/vio_err.py BAG` | VIO error vs ground truth: drift, scale, latency, jumps (sim) |
| `tools/bag_timeline.py BAG --step 1` | state, path clock, pose vs reference, commands, per second |
| `tools/image_stats.py TOPIC...` | is the camera image blank or frozen (live) |

## What to work on next (tuning in Gazebo)

1. **Along-track lag slows the clock.** Gazebo's x500 trails the reference by about 0.2 m, so the clock
   runs at about 0.7 and the course takes 25.5 s on a 19.3 s plan. A separate along-track threshold
   should recover most of that without touching wall clearance.
2. Raise speed in steps: `time_scale` 1.4 → 1.2 → 1.0, then `planner_overrides:=MaxVelMag=1.4`. Keep
   max cross-track under 0.15 m and clearance at or above 0.2 m.
3. Fit the real drone's lag from tonight's bags and make a real-airframe gains file.

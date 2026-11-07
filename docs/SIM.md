# Gazebo sim on GCP

The drone's own stack, in simulation: **Gazebo Harmonic + PX4 v1.16.2 SITL (the drone's firmware) + the
organizers' `skybeta` container** (cuVSLAM, PX4 odometry bridge, flight interface: the same code as the
drone). Our code talks to it exactly as on the drone: `/visual_slam/tracking/odometry` in,
`/skybeta/setpoint/*` out.

```
Gazebo (course from the STEP, Sky Beta model)
  ├─ stereo 1280x800 @30, IMU ──ros_gz_bridge──▶ /drone1/... ──▶ skybeta container: cuVSLAM ─▶ PX4 bridge ─▶ uXRCE agent
  ├─ downward rangefinder, IMU, baro ─────────────────────────────▶ PX4 SITL (EKF2: EV + rangefinder, no GPS)
  └─ ground truth ──▶ /sim/ground_truth/odom   (evaluation only; PX4 never sees it)
your nodes ─▶ /skybeta/setpoint/twist|pose ─▶ flight interface ─▶ PX4 Offboard
sim/pilot.py = the safety pilot (arm, then Position = go signal)
```

## The VM

| | |
|---|---|
| Name | `dronehacks-sim`, project `cs231n-496319`, zone `us-west1-c` |
| Machine | g2-standard-16: 16 vCPU, 64 GB, 1× NVIDIA L4, 300 GB disk |
| Cost | about $1.20/hour while running. **Stop it when nobody is using it.** |

```bash
gcloud compute instances start dronehacks-sim --project cs231n-496319 --zone us-west1-c
gcloud compute ssh dronehacks-sim --project cs231n-496319 --zone us-west1-c
gcloud compute instances stop dronehacks-sim --project cs231n-496319 --zone us-west1-c
```

Teammates need IAM access to the project (Compute OS Admin Login or Instance Admin) to ssh in;
ask Justin. Everyone shares the `~/Dronehacks` checkout; use your own branch and `git pull`.

## Run it

```bash
cd ~/Dronehacks
bash sim/run_sim.sh                      # gz | px4 | bridge | stack | foxglove in tmux "sim"
tmux attach -t sim                       # watch (Ctrl-b n/p switch window, Ctrl-b d detach)
bash sim/scripts/competitor.sh           # flight interface + organizers' example (a square)
python3 sim/pilot.py go                  # waits for EKF position, arms, switches to Position
python3 sim/pilot.py land                # abort
bash sim/run_sim.sh stop
```

Your own launch instead of the example: `bash sim/scripts/competitor.sh <package> <file.launch.py> [args]`.
Keep the flight interface in your launch exactly as `competitor.launch.py` has it.

**Watch it:** Foxglove Studio → open connection → `ws://localhost:8765` through an ssh tunnel:
`gcloud compute ssh dronehacks-sim ... -- -L 8765:localhost:8765`.

## Setup (already done once; rerun is safe)

```bash
bash sim/gcp/setup_vm.sh     # Docker+NVIDIA, ROS 2 Humble, PX4 v1.16.2 + Gazebo Harmonic, ros_gz, skybeta-sim image
bash sim/gcp/setup_sim.sh    # course model from the STEP, skybeta airframe into PX4, rebuild
```

## What's in `sim/`

| Path | What |
|---|---|
| `gz/models/skybeta/` | x500 + OAK-D-like stereo (7.5 cm baseline, 80° HFOV) + downward rangefinder + ground truth |
| `gz/models/dronehacks_course/` | generated from `dronehacks-course-v3.step` by `tools/step_to_gz.py` (gitignored) |
| `gz/worlds/dronehacks_course.sdf` | world frame = planner map frame; dim indoor lighting with shadows |
| `px4/4099_gz_skybeta` | airframe: x500 + the drone's EKF2 setup (rangefinder height, EV pos+yaw, no GPS, no flow) |
| `gz/bridge.yaml` | Gazebo → ROS topic names the organizers' sim launch expects |
| `pilot.py` | safety-pilot stand-in over MAVLink |

## Known differences from the drone (keep in mind when reading results)

- Airframe dynamics are PX4's x500, not Sky Beta. Tune trackers on the drone before trusting gains.
- Cameras are 1280×800 ideal pinhole (the organizers' sim launch is hard-wired to it); the drone is
  1024×640 with real distortion, blur and exposure.
- Course textures are procedural stand-ins. Real walls may have less texture: test with fewer features too.
- No depth topic, same as the drone today.
- The organizers have their own cloud lab (`AI_CONTEXT.md`) with a more realistic scene but a different
  companion stack (ROS 2 Jazzy, direct `/fmu/in` control, no flight interface). Use both.

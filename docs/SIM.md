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

## Two sims, one per workstream

One VM runs two fully isolated sims. Nothing is shared except the GPU.

| | Slot 0: path following | Slot 1: localization |
|---|---|---|
| Working copy on the VM | `~/slot0/Dronehacks` | `~/slot1/Dronehacks` |
| `ROS_DOMAIN_ID` | 10 | 11 |
| Gazebo partition | `sim0` | `sim1` |
| PX4 instance / MAVLink port | 0 / 18570 | 1 / 18571 |
| uXRCE-DDS port | 8888 | 8889 |
| Stack container | `skybeta-sim-0` | `skybeta-sim-1` |
| Foxglove | `ws://localhost:8765` | `ws://localhost:8766` |
| tmux session | `sim0` | `sim1` |

## Workflow from your laptop

```bash
bash sim/gcp/sync.sh 0                                  # copy your checkout to slot 0 (no .git, no builds)
bash sim/gcp/vm.sh 0 'bash sim/run_sim.sh 0'            # start sim 0 (gz | px4 | bridge | stack | foxglove)
bash sim/gcp/vm.sh 0 'bash sim/scripts/competitor.sh'   # flight interface + organizers' example (a square)
bash sim/gcp/vm.sh 0 'python3 sim/pilot.py --sim 0 go'  # waits for EKF position, arms, Position = go
bash sim/gcp/vm.sh 0 'python3 sim/pilot.py --sim 0 land'
bash sim/gcp/vm.sh 0 'bash sim/run_sim.sh 0 stop'
```

Your own launch: `bash sim/scripts/competitor.sh <package> <file.launch.py> [args]`. Build your packages inside the
stack container (`bash sim/scripts/shell.sh` → you are in `/root/team`, your working copy → `colcon build`);
`competitor.sh` sources `/root/team/install` automatically. Keep the flight interface in your launch exactly as
`competitor.launch.py` has it.

Inspect: `bash sim/gcp/vm.sh 0 'tmux capture-pane -p -t sim0:px4 | tail -30'`, or a ROS shell in the slot's domain with
`bash sim/scripts/shell.sh` (in the container) or `bash sim/scripts/shell.sh --host`.

Watch it: Foxglove Studio → `ws://localhost:8765` (slot 1: 8766) through a tunnel:
`gcloud compute ssh dronehacks-sim --project dronehacks-sim-jh --zone <zone> -- -L 8765:localhost:8765`.

## The VM

| | |
|---|---|
| Name | `dronehacks-sim`, project `dronehacks-sim-jh` (account jhollister2025@gmail.com, Debit billing); zone in `sim/gcp/vm.env` |
| Machine | g2-standard-32: 32 vCPU, 128 GB, 1× NVIDIA L4, 300 GB disk (16 vCPU could not run 3 slots at real time) |
| Cost | about $1.80/hour while running. **Stop it when nobody is using it.** |

```bash
gcloud compute instances start dronehacks-sim --project dronehacks-sim-jh --zone <zone>
gcloud compute instances stop  dronehacks-sim --project dronehacks-sim-jh --zone <zone>
```

Teammates need IAM access to the project to ssh in; ask Justin.

## Setup (done once; rerun is safe)

```bash
bash sim/gcp/setup_vm.sh     # NVIDIA EGL, Docker+NVIDIA, ROS 2 Humble, PX4 v1.16.2 + Gazebo Harmonic, ros_gz, skybeta-sim image
bash sim/gcp/setup_sim.sh    # course model from the STEP, skybeta airframe + rcS patch into PX4, rebuild
```

Fixes these scripts apply, in case you rebuild by hand:
- The GCP accelerator image is compute-only: install `libnvidia-gl-<ver>-server` or Gazebo cannot render headless.
- The organizers' `Dockerfile.sim` COLCON_IGNOREs `px4-ros2-interface-lib` by mistake; we build a patched copy.
- PX4 namespaces uXRCE topics for instance > 0 (`/px4_1/fmu/...`); we patch `rcS` so `PX4_NO_UXRCE_NS=1` keeps `/fmu/...`.
- The visual course mesh has per-face vertices for UVs; ODE needs the separate cleaned `collision.stl`.

## What's in `sim/`

| Path | What |
|---|---|
| `gz/models/skybeta/` | x500 + OAK-D-like stereo (7.5 cm baseline, 80° HFOV) + downward rangefinder + ground truth |
| `gz/models/dronehacks_course/` | generated from `dronehacks-course-v3.step` by `tools/step_to_gz.py` (gitignored) |
| `gz/worlds/dronehacks_course.sdf` | world frame = planner map frame; dim indoor lighting with shadows |
| `px4/4099_gz_skybeta` | airframe: x500 + the drone's EKF2 setup (rangefinder height, EV pos+yaw, no GPS, no flow) |
| `gz/bridge.yaml` | Gazebo → ROS topic names the organizers' sim launch expects |
| `scripts/` | one script per process, all driven by `SIM_ID` (`common.sh`) |
| `pilot.py` | safety-pilot stand-in over MAVLink |

## Known differences from the drone (keep in mind when reading results)

- Airframe dynamics are PX4's x500, not Sky Beta. Tune trackers on the drone before trusting gains.
- Cameras are 1280×800 ideal pinhole (the organizers' sim launch is hard-wired to it); the drone is
  1024×640 with real distortion, blur and exposure.
- Course textures are procedural stand-ins. Real walls may have less texture: test with fewer features too.
- No depth topic, same as the drone today.
- The organizers have their own cloud lab (`AI_CONTEXT.md`) with a more realistic scene but a different
  companion stack (ROS 2 Jazzy, direct `/fmu/in` control, no flight interface). Use both.

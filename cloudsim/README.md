# Cloud simulator (our own, on GCP)

A PX4 SITL v1.16.2 + Gazebo Harmonic + ROS 2 Jazzy simulator on one Google Cloud VM, for testing
flight code between lab sessions. Simulation only: it never talks to the drone or a Jetson.

| | |
|---|---|
| VM | `cloudsim`, `g2-standard-8` + 1x NVIDIA L4, us-central1-b, Ubuntu 24.04, 200 GB disk |
| Access | your Google account through IAP + OS Login; no external IP, no keys in the repo |
| Cost | ~$0.85/h while running; ~$20/month for the disk while stopped. $600 budget alerts at 50/90/100% |
| Auto-off | powers off after 30 min with no SSH/noVNC connection (even if the sim is running) |
| Stack | PX4 `v1.16.2` (= drone), Gazebo Harmonic, Micro XRCE-DDS Agent `v2.4.3`, `px4_msgs` `release/1.16` |

## Use it (laptop: Git Bash, macOS or Linux, with `gcloud auth login` done)

```sh
bash cloudsim/vm.sh start                 # BILLS from here
bash cloudsim/vm.sh push                  # copy this checkout's HEAD to ~/Dronehacks on the VM
bash cloudsim/vm.sh ssh                   # shell on the VM
#   on the VM:
bash ~/Dronehacks/cloudsim/sim.sh start   # PX4 + Gazebo (headless) + XRCE agent + /clock bridge
bash ~/Dronehacks/cloudsim/sim.sh status  # PX4 /fmu topics and /clock flowing?
source ~/Dronehacks/cloudsim/env.sh       # ROS + px4_msgs in this shell
bash ~/Dronehacks/cloudsim/sim.sh stop    # kills the world; next start = fresh world (our Reset world)
exit
bash cloudsim/vm.sh stop                  # stop billing for compute. Always do this.
```

`push` **replaces** `~/Dronehacks` on the VM: keep rosbags and notes in `~/runs`, not in the repo.
One person drives the sim at a time; check `sim.sh status` and `tmux ls` before starting.

## Flight-code rules (same as the lab)

- Every run that arms, even here, needs the team's go-ahead and a fresh world (`sim.sh stop` then
  `sim.sh start`). Run passively first (`python3 -m localizer.sim` without the flag).
- `env.sh` sets `FLYX_LAB_ACTIVE=1 FLYX_HITL=0`, the simulator-only guard of `localizer.sim`.
  That is legitimate only on this isolated VM. **Never set them on the drone's Jetson.**
- `/etc/flyx/mission.yaml` on the VM sets `use_sim_time: true`, so `takeoff_land.py` and
  `localizer.sim` run unchanged.
- The default world is PX4's empty world with the stock x500 and GPS. It is not the course and not
  our drone's estimator (that is phases 3 and 4).

## Rebuild from scratch

```sh
bash cloudsim/vm.sh delete && bash cloudsim/vm.sh create
bash cloudsim/vm.sh push && bash cloudsim/vm.sh ssh 'bash ~/Dronehacks/cloudsim/setup.sh'   # ~30-45 min
```

`create` also makes the Cloud Router/NAT (`cloudsim-router`/`cloudsim-nat`) that gives the VM
outbound internet without an external IP. If a zone has no L4s (`ZONE_RESOURCE_POOL_EXHAUSTED`),
retry with `ZONE=us-central1-a` or `-c`.

Teammate access (project owner runs once per person): grant `roles/iap.tunnelResourceAccessor`,
`roles/compute.osAdminLogin` and `roles/compute.viewer` on the project.

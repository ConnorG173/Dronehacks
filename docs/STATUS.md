# Status: 2026-11-08 ~02:20 UTC (sim paused for drone deployment)

## Goal and where we are
Goal: 3 strict-clean sim runs in a row with the course at about 30 s, before trying it on the drone.
Strict-clean means 0 touches at the 0.29 m prop reach, the whole drone through every opening, landing in the pad ring, raw VIO, and a fresh reset each run.

| Requirement | Result |
|---|---|
| Course time | Met: 24.8–25.2 s (candidate 2) |
| Openings 5/5, lands in pad, raw VIO | Met in every finished run |
| 0 touches at 0.29 m | Not met: 1–2 dips of about 0.15 s, min clearance 0.25 m, always at the chicane channel (0.65, -1.12) |
| 3 clean runs in a row | Not met: 0 |

Candidate 3 (path-following 5d62099) targets that one spot.
- Route: `planner/config/course_v3_gateb2.yaml`, with via boxes on the channel centreline.
- Tracker: `tracker_sim_c3.yaml`, which slows the plan where it runs within 0.45 m of a wall.
- Plan: 24.4 s. Kinematic sim: min clearance 0.316 m.
- Runs: le1–le3 in slot 0 and c3rep1–3 in slot 2 (`~/rep3_runs.log`). Their verdicts were pending when the sim was paused.

## What to fly on the drone
- **Deploy profile: `drone-conservative-v1`** (`conservative.launch.py`, Gate B, plan slowed 1.4x, peak ~0.86 m/s).
  Never validated in sim: its only run was during the CPU-overload era.
- **Fast profile:** `fast.launch.py.draft` carries candidate 3's settings. Freeze it as `drone-fast-v1` only after 3 strict-clean runs in a row.
- **Localization:** raw cuVSLAM in, with our jump absorption on. Correction is off for the first flights; see docs/sessions/localization.md.
- **Procedure:** docs/NET_TEST_PLAN.md (bench, carry, hover, first_leg, course). Deploy with `deploy/deploy_to_drone.sh` (skybeta-12 only).
- Measure `base_height_on_pad_m` on the real drone first.

## Where things are
- Branches (local, **not pushed**): `sim-and-plans` (sim, infra, docs), `path-following` (364442d), `localization`.
- Session guides: docs/sessions/path_following.md and docs/sessions/localization.md (each has a "Drone-ready status" section).
- Coordinator run scripts: `sim/coordinator/` (repeat runs, IMU A/B, team-path runs, VIO-vs-GT checks).
- Recordings on the VM: `~/recordings/sim<slot>_<UTC>/*.mcap`. Only runs recorded after fdf3b33 show the 3D view correctly in Foxglove (layout: docs/foxglove_sim_layout.json).

## Not started or paused
- IMU-fusion A/B (`VIO_IMU=1`, `sim/coordinator/ab_imu.sh`).
- Teammates' path (`localizer/data/course_v3_path.bin`) flown with the conservative tracker.
- The questions for organizers in docs/ORGANIZER_QUESTIONS.md (Tailscale, return bonus, checkpoint geometry).

## VM
`dronehacks-sim` (g2-standard-32 + L4, about $1.80/h, project dronehacks-sim-jh). Stop it when idle:
`gcloud compute instances stop dronehacks-sim --project dronehacks-sim-jh --zone us-west1-c`

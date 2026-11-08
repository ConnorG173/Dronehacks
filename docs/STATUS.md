# Status: 2026-11-08 ~02:20 UTC (sim paused for drone deployment)

## Goal and where we are
Goal: 3 strict-clean sim runs in a row with the course at about 30 s, before trying it on the drone.
Strict-clean means 0 touches at the 0.29 m prop reach, the whole drone through every opening, landing in the pad ring, raw VIO, and a fresh reset each run.

| Requirement | Result |
|---|---|
| Course time | Met: 24.8–25.2 s (candidate 2) |
| Openings 5/5, lands in pad, raw VIO | Met in every finished run |
| 0 touches at 0.29 m | Not met: 1–2 dips of about 0.15 s, min clearance 0.25 m, always at the chicane channel (0.65, -1.12). Localization error there is 2.6–3.4 cm, so the cause is planning/tracking margin, not localization |
| 3 clean runs in a row | Not met: 0 |

Candidate 3 (path-following 5d62099) targets that one spot.
- Route: `planner/config/course_v3_gateb2.yaml`, with via boxes on the channel centreline.
- Tracker: `tracker_sim_c3.yaml`, which slows the plan where it runs within 0.45 m of a wall.
- Plan: 24.4 s. Kinematic sim: min clearance 0.316 m.
- Runs: le1–le3 in slot 0 and c3rep1–3 in slot 2 (`~/rep3_runs.log`). Their verdicts were pending when the sim was paused.

**Update 02:40 UTC: `drone-fast-v1` (candidate 3) Gazebo run le2 was strict-clean:**
- 28.0 s by the same course measure as candidate 2's 25.2 s (22.7 s on the node's own clock);
- 0 touches at 0.29 m, min clearance 0.30 m, 5/5 openings, landed in the pad.

le1 and le3 were void: the sim's `pilot.py go` never armed. That arming flake hit 2 of 3 resets and needs fixing before the next sim session.

## What to fly on the drone (ready)
Justin accepted 0.25 m clearance as the bar (the strict 0.29 m was over-conservative), so the fast profile is frozen.

| Order | Profile | Git tag (path-following) | Course |
|---|---|---|---|
| 1 | `conservative.launch.py` | `drone-conservative-v1` | plan ~27 s, peak ~0.86 m/s |
| 2 | `fast.launch.py` = candidate 3 (gateb2 chicane vias, sphere 0.35, near-wall slow-down, drone lag gains 0.66/0.18) | `drone-fast-v1` (8cceade) | ~24–25 s |

Localization: raw cuVSLAM, team_loc M1 (correction off; jump absorption and sanity checks on). See docs/sessions/localization.md and docs/DRONE_RUN.md on the localization branch.

**Deploy steps** (drone on USB-C as `skybeta-usb`; the script refuses any host but skybeta-12):
1. Stop the competitor window's own launch on the drone, then deploy from the sim-and-plans checkout:
   `bash deploy/deploy_to_drone.sh skybeta-usb -- ../Dronehacks-path-following/path_following/team_path_following ../Dronehacks-path-following/planner ../Dronehacks-localization/src/team_loc ../Dronehacks-localization/src/team_loc_matcher`
   The launch files expect `repo_root:=/root/team_ws/src/Dronehacks` (planner + course configs). Follow docs/DRONE_RUN.md (path-following branch) for the exact tree and planner build.
2. Measure `base_height_on_pad_m` (OAK lens height above the floor - 0.0304).
3. Work through docs/NET_TEST_PLAN.md in order: bench, carry (team_loc `ignore_land_detector:=true`), hover, first_leg, course with conservative. Only then fast.
4. After the first flights, fit the real lag (`tools/fit_lag.py <bag> --states course`) before trusting the fast profile. The 0.66/0.18 lag comes from Gazebo's x500, not the Sky Beta.

## Where things are
- Branches (local, **not pushed**): `sim-and-plans` (sim, infra, docs), `path-following` (364442d), `localization` (5304c06; its build and checks are in docs/DRONE_RUN.md on that branch).
- Session guides: docs/sessions/path_following.md and docs/sessions/localization.md (each has a "Drone-ready status" section).
- Coordinator run scripts: `sim/coordinator/` (repeat runs, IMU A/B, team-path runs, VIO-vs-GT checks).
- Recordings: downloaded, finalized and restamped to sim time, in `recordings/` (gitignored), each labelled by run. Open with Foxglove plus `recordings/foxglove_sim_layout.json`. `sim/tools/fix_mcap.py` repairs a recording the recorder didn't close. Originals on the VM: `~/recordings/sim<slot>_<UTC>/*.mcap`. Only runs recorded after fdf3b33 show the 3D view correctly in Foxglove (layout: docs/foxglove_sim_layout.json).

## Not started or paused
- IMU-fusion A/B (`VIO_IMU=1`, `sim/coordinator/ab_imu.sh`).
- Teammates' path (`localizer/data/course_v3_path.bin`) flown with the conservative tracker.
- The questions for organizers in docs/ORGANIZER_QUESTIONS.md (Tailscale, return bonus, checkpoint geometry).

## VM
`dronehacks-sim` (g2-standard-32 + L4, about $1.80/h, project dronehacks-sim-jh). Stop it when idle:
`gcloud compute instances stop dronehacks-sim --project dronehacks-sim-jh --zone us-west1-c`

# DroneHacks · skybeta-12

From a course CAD file to an autonomous indoor flight: a GCOPTER planner turns the mesh into a
time-optimal, validated trajectory, and the drone's flight dashboard flies it on PX4 with live
depth-based route checking.

```
 course CAD (STEP/STL)          planner/                       flight/  (on the drone)
 ───────────────────▶  mesh → occupancy map → RRT per leg  ──▶  dashboard: anchor at the launch
   + start, checkpoints        → safe corridor → GCOPTER         mark, follow the trajectory,
     and goal                  → validate → path.csv / .bin      check it against Depth Anything
                                     │                            depth, PX4 Land, return hop
                                     ▼
                               tools/path_editor.html  (optional hand edits, same format)
```

## Quick start

Everything runs from one [pixi](https://pixi.sh) environment (Python, CadQuery, CMake, Eigen, OMPL):

```sh
pixi run planner-web   # 1. upload the course CAD, place checkpoints, plan  → http://127.0.0.1:8765
pixi run editor        # 2. optional: drag the planned path, export CSV
pixi run demo          # 3. the flight dashboard against a recorded flight  → http://127.0.0.1:8099/?k=demo
pixi run test          # flight logic tests that need no ROS (166 tests)
pixi run test-planner  # planner tests: upstream parity, checkpoints, file format
```

`pixi run plan` plans the course-v3 mission headless from `planner/config/course_v3.yaml`.

## 1 · Mesh → trajectory (`planner/`)

A standalone C++ program around [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) (vendored,
12 of 13 headers byte-identical to upstream; two documented fixes in `geo_utils.hpp`). It voxelises
the course, finds an RRT route per leg through every checkpoint box, builds a convex safe corridor
and optimises a minimum-time trajectory under speed, tilt, thrust and body-rate limits. Every
result is re-checked at 1 ms; a failed plan writes no file. Output: `path.csv` / `path.bin`
(100 Hz, PX4 local NED at the pad) and `route.txt` (map frame). Config keys, formats and frames:
`planner/README.md`. The web front end (`planner/web/`) converts STEP files, shows the course in
3D, streams the planner's progress and draws the RRT routes as they are found.

## 2 · Trajectory → drone (`tools/`)

| | |
|---|---|
| `path_editor.html` | Drag a planned path (top view and height profile), timing kept, CSV out |
| `upload_course.sh` | Upload an edited CSV to the drone as the course (checks start/end moved) |
| `convert_hop.py` | Put a second plan (a return hop) into the first course's frame via the shared map frame |
| `import_path.py`, `import_return.py` | The dashboard API calls behind the two above |
| `dashboard_demo.py` | Serves the real dashboard page with a recorded state; no drone needed |

## 3 · Fly (`flight/`)

What runs on skybeta-12 (Jetson Orin, ROS 2 Humble, PX4 over USB MAVLink, cuVSLAM VIO):

| | |
|---|---|
| `dash/` | Flight dashboard and autopilot: web UI, course following (position or velocity follower), heading control, obstacle guard and in-flight replanning, hole centring, dry run, course + return hop |
| `depth_overlay/` | Depth Anything V2, scaled to metres every frame from stereo; 8 cm obstacle cells per 20 cm height slab |
| `mavlink_usb/` | The single USB MAVLink owner: setpoints, vision, telemetry; an allow-list of operator commands (arm, disarm, Offboard, PX4 Land) |
| `startup/` | Launch files the stack boots with; cuVSLAM camera config |
| `ego/` | Optional EGO-Planner integration (separate container) |
| `courses/` | The routes flown, as planned and as uploaded |

A flight: set home on the launch mark, press **Take off**. The dashboard climbs, follows the
trajectory on its own clock, checks every command against fresh depth, and lands with PX4 Land.
With **course + return hop** it flies a second plan from where it landed. Safety: once airborne,
nothing on the laptop side holds the flight (`DashParams.no_holds`); the RC kill, the dashboard
kill, RC takeover, disarm or lost PX4 position/link stop it.

Tests: `pixi run test` covers the flight logic (course following, replanning, hole control, return
hop, depth geometry). Twelve more modules (`test_motion`, `test_ui`, `test_readiness`, …) need ROS
and `px4_msgs` and run on the drone: `python3 -m unittest discover -p "test_*.py"` in `flight/dash`.

## Also in this repository

- `localizer/`: the team's ROS 2 path-follower package for the simulator (`python3 -m localizer.sim`)
  and an alternative real-drone adapter; its notes are in `localizer/HALF_ASSED.md`.
- `onboard/`, `refnav/`, `main.py`, `learn/`: the lab workspace (`docs/LAB_WORKSPACE.md`).
- `docs/deploy_notes/`: dated deploy and bench records from the drone.

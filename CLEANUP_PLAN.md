# Cleanup plan

Inputs: the overnight brief, baseline in `CLEANUP_REPORT.md` §1, and a mechanical inventory
(g++ -MM include closure, AST import lists, git grep for each filename, setup.py/launch references).

## Target tree

```
README.md                      lab file; gains a team section (layout, run in sim, run on drone, tests)
CLEANUP_PLAN.md, CLEANUP_REPORT.md
.gitignore                     + __pycache__/, *.pyc, .pytest_cache/, colcon build/ install/ log/
dronehacks-course-v3.step      kept (source of planner/maps/course_v3.bin), see ASK
<lab scaffolding untouched>    AI_CONTEXT.md AGENTS.md CLAUDE.md CONNECT_FROM_LAPTOP.md FLIGHT_CODE.md
                               SIM_SENSORS.md SIM_SENSOR_EXAMPLE.py onboard/ refnav/ learn/ tests/
                               main.py flight.yaml route.json *-source.json .onboard/ .vscode/
planner/                       unchanged except: tools/ trimmed, README layout updated
  CMakeLists.txt src/main.cpp tests/{run_tests.py,upstream_demo.cpp}
  third_party/gcopter/ (all 13 headers: every one is in the include closure of both targets)
  config/{course.yaml,course_v3.yaml} maps/{course_v3.bin,wall_demo.bin}
  tools/{cad_to_map.py (map provenance), make_wall_map.py (imported by run_tests.py)}
localizer/                     ROS 2 ament_python package "localizer" (name kept, see ASK)
  package.xml setup.py setup.cfg resource/localizer
  localizer/__init__.py
  localizer/core/              PURE Python + numpy, no ROS import (pytest on the laptop)
    __init__.py
    path.py                    load_path, CoursePath, path_to_course, time_scale_for   (from streamer.py)
    frames.py                  wrap, Transform2D (course<-local), pad_init, NED<->ENU, quaternion
                               helpers (from bev.py), fence inside/clamp            (from streamer.py, bev.py)
    clock.py                   PathClock + Setpoint                                  (from streamer.py)
    follower.py                Config, VehicleState, Command, Follower.step(now, state) -> Command
                               WAIT -> READY -> TAKEOFF -> FOLLOW -> FINISH, HOLD on fault,
                               STOPPED when Offboard is lost              (lifted from path_streamer.py)
  localizer/sim.py             adapter: px4_msgs, /fmu/in/* direct, full feedforward, self-arm only
                               with --i-have-team-approval (+ FLYX_LAB_ACTIVE=1 guard), NAV_LAND
  localizer/real.py            adapter: /skybeta/state + odom -> core; publishes ONLY the lead point,
                               to /localizer/setpoint_preview by default or /skybeta/setpoint/pose
                               with live:=true. Never arms, never /fmu/in   (git mv of path_streamer.py)
  launch/real.launch.py        (git mv of localizer.launch.py, matcher removed)
  launch/sim.launch.py
  data/course_v3_path.bin      flown path + test fixture (pinned: the planner is not deterministic)
  test/px4_model.py            kept unchanged
  test/closed_loop.py          offline PX4-model sim driving the Follower core (sim + real modes),
                               runnable as a script for the table + PNG     (replaces tools/sim_streamer.py)
  test/test_core.py            ported test_streamer.py + ported attitude tests from test_bev.py
  test/test_follower.py        state machine: fault->hold, end->land request, offboard lost->stop, ...
  test/test_adapters.py        conversion functions with fake messages (no ROS), checked against
                               flight_interface's ENU->NED (y, x, -z), yaw_ned = pi/2 - yaw_enu
```

## Interfaces (core)

- `VehicleState` (adapter fills from messages; all in the vehicle's **local ENU** frame =
  the convention flight_interface uses, x_enu = y_ned, y_enu = x_ned, z up, yaw CCW = pi/2 - yaw_ned):
  `pose_stamp, pos (3,), yaw, estimate_ok, status_stamp, armed, landed, offboard,
  odom_tilt_deg?, imu_tilt_deg?`
- `Command`: `mode, p, v, a, yaw (None = don't command), yaw_rate, q (lead point), publish (bool),
  request_offboard_arm (bool), request_land (bool), reason` - all local ENU.
- `Config` dataclass: path/pad/fence/clock/takeoff parameters plus the environment differences as
  values: `self_arm`, `land_at_end`, pad-check limits (None = off), `z_from_pad`, `dry_run`.
  Each adapter builds its own Config defaults; the core has no `if sim`.

## Where environment differences live

| Difference | sim | real | Lives in |
|---|---|---|---|
| QoS | PX4 BEST_EFFORT + TRANSIENT_LOCAL depth 1 | sensor_data / default String / PX4_QOS | adapter |
| time source | node clock (use_sim_time via params file, like takeoff_land) | node clock (system) | adapter passes `now` |
| feedforward vs lead | publishes p, v, a, yaw, yaw_rate | publishes q, yaw | adapter picks fields |
| who arms | core requests (Config.self_arm, approval flag) -> adapter sends VehicleCommand | pilot on RC | Config + adapter |
| end of path | Config.land_at_end -> NAV_LAND | hold goal, pilot lands | Config + adapter |
| pad checks | off (PX4 local frame is not the pad frame) | odom near 0, yaw near 0, tilt vs IMU | Config |
| z reference | z_from_pad (PX4 local z on the pad) | absolute (current behaviour: height above floor) | Config |

## File-by-file disposition

| Path | Action | Evidence / reason |
|---|---|---|
| planner/tools/plot_path.py | DELETE | git grep: referenced only by itself and view_path.py; not in tests/README/run path |
| planner/tools/view_path.py, view_path.html | DELETE | referenced only by each other |
| planner/tools/__pycache__/*.pyc (2) | DELETE (untrack) + gitignore | committed bytecode |
| planner/tools/cad_to_map.py | KEEP | only way to regenerate maps/course_v3.bin from the STEP (command in course_v3.yaml) |
| planner/tools/make_wall_map.py | KEEP | imported by tests/run_tests.py |
| planner/config/course.yaml, maps/wall_demo.bin | KEEP (ASK) | demo config the README documents; not used by tests |
| planner/* everything else | KEEP | CMake targets, include closure, tests |
| localizer/localizer/streamer.py | SPLIT -> core/path.py, frames.py, clock.py (git mv to core/clock.py) | follower math |
| localizer/localizer/path_streamer.py | git mv -> real.py; state machine lifted to core/follower.py | real run path |
| localizer/localizer/bev.py | DELETE after copying quat_to_mat, px4_to_enu_flu, tilt_deg, yaw_of to core/frames.py | rest is matcher-only |
| localizer/localizer/course_matcher.py, match.py, depth.py | DELETE | only matcher entry point + their own tests import them; /localizer/fix consumed by nothing |
| localizer/test/test_match.py, synth.py, test_bev.py, test_depth.py, test_cad_to_wallgrid.py | DELETE | test deleted code (2 attitude tests of test_bev.py ported) |
| localizer/tools/cad_to_wallgrid.py | DELETE | wall-grid generator; follower needs only 3 scalars from the npz |
| localizer/tools/sim_streamer.py | DELETE (replaced by test/closed_loop.py driving the core) | one-off comparison incl. the OldCarrot remnant |
| localizer/data/course_v3_wallgrid.npz, .png | DELETE | npz supplied only start_xy, start_heading_rad, cage_half_m -> Config parameters |
| localizer/data/course_v3_path.bin | KEEP | default path + test fixture |
| localizer/test/px4_model.py | KEEP | closed-loop evidence |
| localizer/test/test_streamer.py | git mv -> test/test_core.py, ported | |
| localizer/launch/localizer.launch.py | git mv -> launch/real.launch.py, matcher removed | |
| localizer/BRIEF.md | DELETE (facts that still matter go into localizer/README.md) | planning brief for the matcher; hard rules restated in README |
| localizer/HALF_ASSED.md | KEEP, trimmed to the follower (course_matcher section removed, Carrot wording updated) | live list of known shortcuts |
| localizer/.gitignore | DELETE, folded into root .gitignore | |
| onboard/**/__pycache__/*.pyc (15) | untrack only (`git rm --cached`) + gitignore | brief allows it |

## Verification per step
- planner: rebuild after deletion, obj sha256 must equal baseline, course_v3 plan succeeds, run_tests.py.
- core/adapters: pytest, `py -m py_compile` on every .py, adapter conversion tests with fakes,
  closed-loop sim through the core in both modes (table + PNG).
- independent review agent on the final tree; fix; re-run everything.

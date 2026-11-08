# Net test plan: drone 12 tonight (Sat Nov 7)

Goal: by the first scored block (posted 7:30 PM, scored runs from 10 PM), have **one configuration that
finishes the course reliably**: all 5 checkpoints, landing in the pad ring, Gate B route, conservative speed.
A finish at any speed beats every unfinished run; speed comes after.

Exact package names, launch commands and presets come from each workstream's `docs/DRONE_RUN.md`
(path following: branch `path-following`; localization: branch `localization`).

## Roles

- **Safety pilot** (attends the net safety briefing; rule 8.1): RC in hand, thumb on the kill switch, arms and
  flips to Position = go. Takes over with sticks or the mode switch at any doubt.
- **Operator**: laptop on the drone's Wi-Fi (or USB), runs the launch and the logging, calls the flight.
- **Watcher**: eyes on the drone, calls walls and drift.

Never arm outside the closed net. Props off on the bench. One drone in the net at a time.

## 0. Before going to the net (laptop, ~20 min)

1. Drone powered on the bench, props off, still for the first minute (VIO origin = power-up pose).
2. Connect: USB-C (`ssh skybeta-usb`) or same Wi-Fi (`ssh dronehacks@<ip>`; `ip -4 -br addr show wlP1p1s0`
   over USB tells you the IP). Run `hostname` and check it says **skybeta-12**.
3. Health: `cd ~/skybeta && host/doctor.sh` must show 0 FAIL.
4. Deploy: from the repo root on the laptop, `bash deploy/deploy_to_drone.sh <target> -- <package dirs>`
   (dirs listed in the DRONE_RUN.md files). The build must finish with no failed packages.
5. Stop the organizers' example in the `competitor` window (attach, select the window, Ctrl-C): two flight
   interfaces fight. Then start our launch by hand in a stack shell (`host/run.sh shell`). Optional, for a
   permanent setup: set `COMPETITOR_PKG` / `COMPETITOR_LAUNCH` in `/etc/skybeta/drone.env` (sudo) and restart
   the stack.

## 1. Bench check (props off, ~15 min)

- `ros2 topic hz /visual_slam/tracking/odometry` ≈ 30 Hz; `/visual_slam/status` vo_state 1.
- Our launch up: planner reports a valid plan; `/team/mission/state` = idle; `/skybeta/state` shows the
  envelope; no setpoints leave the tracker before go.
- Localization: `/team/odom_map` on the pad ≈ the launch mark in `course` coordinates; `/team/loc/status` OK.
- **Carry test** (needs its own localization launch): while disarmed PX4 reports *landed*, so the default
  `team_loc` re-anchors to the launch pose whenever the drone is held still and the test would look perfect.
  Launch `team_loc` with `ignore_land_detector:=true`, then pick the drone up and walk it 1 m forward, 1 m left,
  back: `/team/odom_map` must follow (catches a frozen or mirrored VIO before it flies). Then put it back on the
  launch mark facing the start heading and **relaunch `team_loc` with the defaults** before flying.
- `base_height_on_pad_m`: measure with a tape, floor to the Pixhawk IMU with the drone on the pad. Both
  workstreams take this launch arg.

## 2. Flights (in the net, safety pilot ready)

Start each run: drone on the launch mark at the planner's start heading → launch running, plan valid →
pilot arms in Altitude → flips to Position → flight interface enters Offboard → our node flies.
Record each run (operator): `ros2 bag record -o run_NN /team/odom_map /team/mission/state /team/loc/status
/visual_slam/tracking/odometry /visual_slam/status /fmu/out/vehicle_local_position /fmu/out/vehicle_status_v1
/skybeta/state /skybeta/setpoint/twist`. Bags are for analysis only: never feed run data into a later run (6.4).

| # | preset | pass criteria | if it fails |
|---|---|---|---|
| 1 | hover: 1.0 m, 10 s, land | holds within ~0.2 m, no oscillation, lands on the spot | pilot takes over; check VIO latency / EKF2_EV_DELAY, tracker gains |
| 2 | first leg at ~0.4 m/s | reaches checkpoint 1 without touching, hovers, lands | lower speed; check frames (course vs drone heading at start) |
| 3 | full course, Gate B, conservative speed | 5/5 checkpoints, lands in the pad ring | stop at the failing segment; fix there; repeat |
| 4+ | repeat #3 | same result twice in a row | — |
| then | raise speed one step at a time | still 5/5 | back off one step |

Abort: pilot flips the mode switch (Offboard does not come back until disarm + re-arm) or kills on the ground.
After any wall touch or tracking loss, land, reset on the launch mark, restart the stack only on the ground.

## 3. Lock the scored-run config

Write the exact commit, launch line and preset that passed #3 twice into this file before the first scored
block. Push the team repo (rule 9.3: link due Sun 12:30 PM, code flown must be pushed by Sun 4 PM).

## Known risks to watch

- **Window**: VIO broke passing the window in sim (the wall fills the view; OAK stereo depth fails < 0.35 m).
  Gate B route tonight; window only after Gate B is reliable.
- **Flying above the walls**: the rangefinder sees the wall tops and PX4's height jumps. Stay below 1.6 m.
- **VIO latency** on the drone: p50 60–160 ms, p95 up to 340 ms under load, vs EKF2_EV_DELAY 70. Watch for
  oscillation in the hover test.
- **Dim tunnel**: fine in sim (VIO error < 5 cm); unknown on the real course.
- **Return bonus**: off (needs organizer ruling; docs/ORGANIZER_QUESTIONS.md).

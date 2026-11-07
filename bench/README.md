# Bench and tethered tests (no RC transmitter)

`bench.py` runs **on the Jetson**, started from an SSH session on your laptop.
Everything here is self-contained in this `bench/` folder: run every command from it
(`cd bench`), and logs go to `bench/bench_logs/` (git-ignored). It talks
to PX4 over the same uXRCE-DDS / `px4_msgs` interface as `refnav/` and
`onboard/examples/takeoff_land.py`, and adds nothing on the PX4 side.

```
laptop ──SSH──> Jetson: bench.py ──ROS 2 / uXRCE-DDS──> Pixhawk 6C (PX4)
                  └─ VIO (cuVSLAM + ev_bridge) ──/fmu/in/vehicle_visual_odometry──┘
```

Without an RC transmitter there is **no pilot takeover**. Your safety layers are:

1. **The tether**, which physically limits where the drone can go.
2. **`bench.py`'s guard**, which lands or kills on fence, ceiling, tilt, tracking error,
   estimator loss, stale telemetry, low battery or a time limit.
3. **The operator keys** in the SSH terminal, plus `python3 bench.py kill` in a second terminal.
4. **PX4's own failsafes**, if the Jetson program or the Jetson itself dies.
5. **A physical battery disconnect** that someone can reach. Get one: an inline XT60
   loop key or anti-spark switch. Layers 2–4 all depend on the Jetson→Pixhawk link.

## 1. One-time setup

### Jetson

- Micro XRCE-DDS Agent on the Pixhawk serial link, e.g.
  `MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600`, with matching `UXRCE_DDS_CFG`
  and baud on the Pixhawk side (TELEM2).
- ROS 2 with `px4_msgs` built **for your PX4 firmware version**. Topic names (`_v1`
  suffixes) and fields come from it.
- The VIO stack from `onboard/` running and publishing `/estimator/healthy`. Without
  VIO, pass `--no-health` and make sure PX4 has some other position source.

### SSH must notice a dead link quickly

If the Wi-Fi drops, `bench.py` lands when its SSH session dies (SIGHUP). By default
sshd can take many minutes to notice. In `/etc/ssh/sshd_config` on the Jetson:

```
ClientAliveInterval 1
ClientAliveCountMax 3
```

Then `sudo systemctl restart ssh`. A dead link now ends the session within about 3 s.

**Do not run flight tests inside tmux or screen.** They keep the program alive when SSH
drops, so the landing never triggers. `bench.py` refuses to fly inside them unless you
pass `--allow-multiplexer`.

### PX4 parameters (set with QGroundControl over USB)

Check each name and value against the parameter reference for your firmware version.

| Parameter | Value | Why |
|---|---|---|
| `COM_RC_IN_MODE` | 4 (stick input disabled) | Fly without an RC transmitter |
| `COM_OF_LOSS_T` | 0.5 | Seconds without setpoints before the Offboard-loss failsafe |
| `COM_OBL_RC_ACT` | Land | What PX4 does when the Jetson stops streaming |
| `COM_DISARM_LAND` | 2 | Auto-disarm 2 s after landing |
| `COM_DISARM_PRFLT` | 10 | Auto-disarm if armed but never takes off |
| `COM_MOT_TEST_EN` | 1 | Needed by `bench.py motors` |
| `COM_ARM_WO_GPS` | allow | Indoors, with vision only |
| `NAV_DLL_ACT` | 0, or Land | Only matters if a GCS was connected and is then unplugged |
| `MPC_XY_VEL_MAX` | 1.0 | Keep the tethered tests slow |
| `MPC_Z_VEL_MAX_UP` / `_DN` | 0.5 / 0.5 | |
| `MPC_TILTMAX_AIR` | 20 | |
| `MPC_THR_HOVER` | measured | Check after the first hover in the log |

Also confirm the EKF2 external-vision settings (`EKF2_EV_CTRL`, `EKF2_HGT_REF`,
`EKF2_EV_DELAY`, `EKF2_EV_POS_X/Y/Z`) and the actuator geometry and motor order in
QGroundControl's Actuators page.

### Tether rig

- Anchor to something heavy (a weight plate or sandbags), centred under the drone.
- Attach to the bottom centre of the frame, so a taut tether does not tip the drone.
- Length: the test height plus about 0.2 m. The default hover is 0.4 m, so roughly
  0.6 m. The drone must not be able to reach people, walls or the tether's anchor
  with its props.
- Use a rope or strap the props cannot pull in. Keep it clear of the camera's view.
- People stay outside prop reach, with eye protection. One operator at the laptop,
  one person by the battery disconnect.

## 2. Test order

Run each test in order, and only move on when the previous one passes. Every command
exits with code 0 on pass. Flight logs go to `bench_logs/` (CSV plus a JSON summary);
also pull the PX4 ULog after every armed test.

### Props OFF

| # | Command | What it checks | Pass |
|---|---|---|---|
| 1 | `python3 bench.py link` | All PX4/VIO topics arrive at sane rates; PX4 mode, arming checks, failsafe flags, EKF2 fusion, battery, timesync | Required topics present |
| 2 | `python3 bench.py estimator --cover` | 30 s standing still: health, local position valid, vision fused, VIO rate, noise, drift; then cover the camera | Health ≥ 99 %, noise ≤ 1 cm, drift ≤ 5 cm, goes unhealthy when covered and recovers |
| 3 | `python3 bench.py frames` | Carry it forward, right and up, then rotate 90° clockwise | Each move appears on the right axis with the right sign; VIO and PX4 distances agree |
| 4 | `python3 bench.py motors` | Spins each motor alone at 10 % for 2 s; you confirm position and direction | Every motor matches the expected layout |
| 5 | `python3 bench.py arm` | Arms in Offboard three times: normal disarm, **KILL**, then stops streaming to trigger the Offboard-loss failsafe | All three behave; KILL disarms within a second |

`frames` catches the classic VIO frame mistakes: a flipped axis, swapped axes, or a
rotated frame. Fix those before arming. With a bad frame, PX4 pushes the drone the
wrong way and the position loop runs away.

`arm` is the most important props-off test. It proves the emergency stop works before
you need it, and it shows what PX4 does on its own when the Jetson program dies.

### Props ON, tethered

During a flight: **Space** or **l** lands, **k** kills (motors stop, the drone drops),
**Ctrl-C** lands and a second **Ctrl-C** kills. Losing SSH lands. Keep a second SSH
terminal open with `python3 bench.py kill` typed, ready to press Enter.

| # | Command | What it checks | Pass |
|---|---|---|---|
| 6 | `python3 bench.py hover` | Take off to 0.4 m, hold 10 s, land | Lands cleanly; tracking error a few cm; no guard triggers |
| 7 | `python3 bench.py steps` | ±0.2 m forward and right, 0.15 m up, 30° yaw, each held 4 s (smooth moves with feedforward) | Rise time, overshoot, settle and steady-state error per axis look reasonable (see below) |
| 8 | `python3 bench.py steps --shape step` | The same with instant setpoint jumps: the classic tuning step response | Overshoot ≲ 20 %, no oscillation |
| 9 | `python3 bench.py circle` | 0.2 m radius circle, 6 s laps, with velocity and acceleration feedforward | Tracking RMS a few cm, small lag |
| 10 | `python3 bench.py path --file ../planner/out/path.bin --speed 0.3` | A planner trajectory, slowed down | Only if the whole path fits in the fence; refuses otherwise |

For a long planner path (`../planner/out/path.bin` from this folder), `path` will refuse: the tether area is too small. Plan a short
test course, or extract a small segment, to check the planner-to-PX4 pipeline here.

Each flight prints its plan, checks it fits inside the fence and ceiling with 0.1 m
margin, refuses if it does not, then asks you to type `FLY`.

### Reading the results

- **Hover drifting or circling slowly:** estimator problem (frames, `EKF2_EV_DELAY`,
  vibration), not tuning.
- **Steady offset in hover height:** hover thrust is wrong. Check `MPC_THR_HOVER` and
  the hover thrust estimate in the ULog.
- **`steps`: slow rise with no overshoot:** position or velocity gains are low.
  **Overshoot or oscillation:** gains too high, or rate/attitude loops not tuned. Tune
  from the inside out (rate, then attitude, then velocity, then position).
- **`circle`: tracking lag well above 100 ms, or RMS growing with speed:** feedforward
  is not taking effect, or the limits are saturating (`MPC_XY_VEL_MAX`,
  `MPC_ACC_HOR_MAX`, `MPC_TILTMAX_AIR`).
- **Tracking error grows suddenly:** the tether went taut. Shorten the test or lengthen
  the tether slightly.

## 3. Guard limits

Defaults are sized for a short tether at low height. All are options on the flight
commands (`python3 bench.py hover --help`).

| Condition | Default | Action |
|---|---|---|
| Horizontal distance from takeoff | > `--fence` 0.5 m | Land |
| Height above takeoff | > `--ceiling` 1.0 m | Land |
| Fence + 0.3 m, or ceiling + 0.3 m | | **Kill** |
| Tilt | > `--max-tilt` 30° | Land |
| Tilt | > 60° | **Kill** |
| Tracking error | > `--max-track-err` 0.35 m for 0.4 s | Land |
| Estimator health (`/estimator/healthy`) | false or silent for 0.5 s | Land |
| PX4 position or status telemetry | older than 0.5 s | Land |
| Battery | < `--min-battery-land` 20 % (needs ≥ 50 % to arm) | Land |
| Airborne time | > `--max-flight-s` 120 s | Land |

**Kill** force-disarms. On a tether at 0.4 m, dropping the drone is the lesser harm than
letting it fly away. That trade is not true at real altitude, which is why these limits
are not meant for free flight.

If PX4 leaves Offboard on its own (one of its failsafes, or `bench.py land` from another
terminal), `bench.py` stops streaming and does not fight it. The **k** key still works.

## 4. Code

| File | Responsibility |
|---|---|
| `bench.py` | Command-line entry point and options |
| `benchtest/procedures.py` | Each test, its prompts and its pass/fail checks |
| `benchtest/sequencer.py` | Takeoff, run a profile, land: the state machine (pure) |
| `benchtest/guard.py` | The safety envelope (pure) |
| `benchtest/profiles.py` | Hover, steps, circle and planner-path setpoints with velocity and acceleration (pure) |
| `benchtest/analysis.py` | Tracking error, lag, step response, drift and frame checks (pure) |
| `benchtest/link.py` | ROS 2: PX4 and VIO subscriptions, setpoints, commands |
| `benchtest/console.py` | Operator keys, signals, SSH-loss handling |
| `tests/test_benchtest.py` | Unit tests with a kinematic fake PX4 (`tests/conftest.py` sets the import path) |

`python3 -m pytest -q tests` (from `bench/`) runs them. Like the refnav tests in the
repo's top-level `tests/`, they test decisions, not flight dynamics.

These tools have been unit-tested and run end to end against a stubbed ROS layer with a
fake vehicle. They have **not** yet run against a real PX4. Expect to fix things
during the first `link` and `estimator` runs (topic names, message fields, rates) before
anything arms. Try them in SITL first if you can.

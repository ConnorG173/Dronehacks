# drone-conservative-v1 on drone 12's dashboard stack

> **Route (2026-11-07 22:30):** the course changed. By default `run.sh` flies the team's new route,
> `repo/path_following/routes/course_20261107_2212.bin`, the same file the dashboard loaded at 22:14.
> It's slowed uniformly to the conservative 0.86 m/s peak: 9.0 s becomes about 25.5 s. It ends 0.4 m above the pad,
> then lands. It's flown relative to the drone's heading at arming, so **place the drone on the mark facing
> the same way as for the dashboard's copy of this route.**
> - Offline check: done, cross-track 2.6 cm RMS (max 14 cm), landed 2 cm from the route end.
> - No sim check: Gazebo only has the old walls.
> - `first_leg` only exists for the old Gate B route (`bash run.sh first_leg <bh> gateb`).

> **Heading fix (2026-11-08):** PX4 may now fuse the gyro heading-corrected VIO (`/team_heading/odometry`,
> enabled by the marker `/root/team_ws/claude/mavlink_usb/heading_fix_enabled`). `run.sh` reads the
> bridge's `input_topic` and steers on exactly the topic PX4 fuses. It prints `pose source = ...`, so check
> that line. If the tracker steered on a different heading than PX4, its velocity commands would rotate by
> the correction, which reaches +15–25° per lap.
> - **Before the pilot arms with the heading fix on:** run `bash calibrate.sh`. The drone must sit still on
>   the pad, disarmed. The script waits until it prints `calibrated`.
> - It sends only a calibration request, not the dashboard's Reset button, which also sends a Hold.
> - Pressing Reset instead is safe: our first twist clears that Hold.

The same flight as `drone-conservative-v1`, adapted to the stack now running on drone 12. That flight is:
- Gate B route;
- plan MaxVelMag 1.2, slowed 1.4x (peak about 0.86 m/s, course about 27 s);
- lag 0.66/0.18;
- z ceiling 1.6 m.

The safety pilot is in charge the whole time.

## How it fits the current drone

| On the drone now (teammates' dashboard) | What this does |
|---|---|
| The competitor window runs `mavlink_usb/dashboard.launch.py`: the flight interface, the dashboard, the USB MAVLink adapter (which carries vision and setpoints to PX4), a depth worker and the Offboard selector | Leaves all of it running. Never stop that launch: the vision feed to PX4 lives in it. |
| The flight interface follows whichever setpoint arrived last (pose or twist), max 1.0 m/s, altitude 0.3–2.0 m | Starts only `mission_tracker` (no second interface). It publishes `/skybeta/setpoint/twist` into the dashboard's interface. |
| Leveled cuVSLAM on `/claude/vio_leveled/odometry` is what PX4 fuses | Uses the same topic as its pose. |
| Dashboard Hold / Land / KILL | Still work. Our node publishes nothing until the drone is armed in Offboard. The moment the dashboard sends any pose setpoint, or the interface reports Hold, it stops for good. KILL disarms, and our node then aborts. |

## One-time setup (on the ground, disarmed)

```bash
ssh skybeta
cd ~/team_ws/team_staging/conservative_v1/kit
bash build.sh        # package + planner into the kit's own build/install (about 2-5 min of CPU; never while flying)
```

## Each flight

1. **Place the drone** centred on the launch mark, nose +x (into the maze). The course frame is the pose at arming.
2. **Measure `base_height_on_pad_m`** once: floor to OAK lens centre, minus 0.0304 m. Expect 0.05–0.12.
3. **Release the dashboard.** Don't take control or start a course in it. Leave it open to watch, and for Hold, Land and KILL.
4. **Preflight (read-only):** `bash preflight.sh`. You want all of these:
   - planner and package OK;
   - exactly 1 flight_interface;
   - no mission_tracker running;
   - VIO at 30–60 Hz;
   - no `/skybeta/setpoint/pose`;
   - PX4 local position flowing;
   - the topic PX4 fuses, and if the heading fix is on, `calibrated: true`.

   With the heading fix on, run `bash calibrate.sh` last, with the drone still on the pad.
5. **Start:** `bash run.sh hover <base_height>`, later `bash run.sh course <base_height>`. Then `tail -f ~/team_ws/bags/cv1_hover_*.log` until you see `plan ready`. Nothing moves yet.
6. **Offboard:** `/skybeta/state` must show `nav_state_name: OFFBOARD`. The dashboard's grounded selector does this at startup. If PX4 is in another mode (for example AUTO_LOITER after a landing), the pilot or operator puts it in Offboard while disarmed. **Our code never changes modes or arms.**
7. **Pilot arms.** Our node takes off at once: 1 m/s climb to 1.0 m. Then:
   - `hover`: holds 10 s and lands on the mark.
   - `first_leg`: flies slowly (0.4 m/s) to the first maze gap, holds 5 s and lands there.
   - `course`: flies the full Gate B route and lands on the pad.

   PX4 auto-disarms after touchdown. The log shows the mission state: `idle → takeoff → course → landing → landed → done`.
8. **Abort, any of these:**
   - pilot RC takeover (mode switch);
   - dashboard Hold or Land;
   - dashboard KILL (motors off);
   - `bash stop.sh`. The interface then holds position after 0.5 s.

   After a takeover our node never re-engages in that run.
9. **Stop:** `bash stop.sh`. It closes our node and the recorder; the dashboard is untouched. Bags go to `~/team_ws/bags/cv1_<preset>_<time>/`.

Fly in order: hover, then course. With the old route: hover, then first_leg, then course. Move on only after a clean flight. After the first flights, fit the real airframe lag before going faster:
`python3 repo/path_following/tools/fit_lag.py <bag> --states course`

## What our node will do on its own

| Situation | Behaviour |
|---|---|
| `/skybeta/state` older than 0.5 s, or `px4_link` false | No authority: in flight it aborts and goes silent |
| Pose older than 0.25 s (measured from the VIO timestamp) | Zero velocity: PX4 brakes and holds |
| Pose jump (VIO relocalisation) | Aborts. While Offboard lasts it descends in place; once Offboard ends it stops commanding for good |
| A dashboard pose setpoint, or interface Hold | Yields: publishes nothing from then on |
| Planner fails | Publishes nothing: arming gives no autonomy |

## Verified in Gazebo (old course, Gate B route, 2026-11-08)

Setup for these runs:
- Dashboard mode, with a stand-in flight interface.
- PX4 put in Offboard while disarmed, then armed (`pilot.py go_offboard`), as on the drone.

Results:
- **cv4a, full course:** done, course 31.6 s, 0 touches at 0.25 m, 5/5 openings, landed 3.8 cm from pad centre, cross-track 6.6 cm average.
- **cv2b, back-off test:** a pose injected mid-course gives `YIELDED to dashboard pose setpoint`, then the abort; nothing is published after.

## Not verified on hardware

- Lag 0.66/0.18 comes from Gazebo's x500, not the Sky Beta.
- In Gazebo this ran with the organizers' stock interface standing in for the dashboard's interface, not with the USB MAVLink link.
- Offboard entry depends on the dashboard's selector or the pilot.
- Not flown on the heading-fixed pose (`/team_heading/odometry`) in Gazebo. It's the same message type and
  timestamps as the levelled VIO; only yaw differs, and the launch pose captured at arming absorbs any offset.

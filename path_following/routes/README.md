# Routes flown from a file

| File | Source | sha256 |
|---|---|---|
| `course_20261107_2212.bin` | Teammates' route for the changed course, from drone 12 `~/team_ws/claude/paths/course_20261107_2212/path.bin` (2026-11-07 22:14 PST). No CAD of the new walls was published; the team confirmed it clears them. | `6c0f068c6332ecce1670ecf5d2d54b3a54a822485549b3549c1f69be7d39bcbd` |

Format: DHPATH v1 (see `planner/`). PX4 local NED, launch-relative: origin on the launch mark,
+x = the direction the drone faces there, +y to its right, z down.

**course_20261107_2212:**
- 8.99 s, 13.6 m, peak 2.44 m/s.
- Starts at rest 1.0 m above the mark.
- Ends 0.4 m above the pad, at (-1.685 fwd, +0.586 right) from the mark.

`dashboard.launch.py path_file:=...` slows it uniformly to the conservative 0.86 m/s peak.
It flies relative to the drone's heading at arming, so place the drone exactly as for the dashboard's copy of this route.

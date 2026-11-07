"""Simulator ground truth logger (DIAGNOSTIC, simulation only; never commands the drone).

Prints and records the simulated vehicle's TRUE pose from dronehacks_sim (SIM_SENSORS.md), to
compare with the follower's estimate (`est_course` in the sim adapter's log, and
/fmu/out/vehicle_local_position) and to check the spawn pose against the launch mark.
Flight code must never read this; it is the oracle.

    python3 localizer/tools/sim_truth.py --seconds 120 --csv truth.csv
"""
import argparse
import csv
import math
import time

PAD = (-2.5146, 0.9144)           # planner/config/course_v3.yaml StartPos, heading +x


def yaw_of_xyzw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    from dronehacks_sim import OutsideMap, Simulation, Unavailable, WorldChanged
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    out = open(a.csv, "w", newline="") if a.csv else None
    w = csv.writer(out) if out else None
    if w:
        w.writerow(["wall_s", "sim_time_ns", "x_enu", "y_enu", "z_enu", "yaw_enu_deg"])
    with Simulation() as sim:
        first = True
        until = time.monotonic() + a.seconds
        while time.monotonic() < until:
            try:
                pose = sim.pose()
            except WorldChanged:
                print("World reset: restart this logger.")
                break
            except (Unavailable, OutsideMap) as e:
                print(f"unavailable: {e}")
                time.sleep(1.0 / a.hz)
                continue
            x, y, z = (float(v) for v in pose.position_enu_m)
            yaw = math.degrees(yaw_of_xyzw(pose.orientation_xyzw))
            if first:
                d = math.hypot(x - PAD[0], y - PAD[1])
                print(f"start: ({x:.3f}, {y:.3f}, {z:.3f}) yaw {yaw:.1f} deg; launch mark ({PAD[0]}, {PAD[1]}) "
                      f"heading 0 deg -> offset {d:.3f} m, heading error {yaw:.1f} deg")
                first = False
            print(f"{pose.sim_time_ns * 1e-9:9.2f}  truth ({x:6.3f}, {y:6.3f}, {z:5.2f})  yaw {yaw:6.1f}", flush=True)
            if w:
                w.writerow([f"{time.time():.3f}", pose.sim_time_ns, f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", f"{yaw:.2f}"])
            time.sleep(1.0 / a.hz)
    if out:
        out.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Print EKF2 accel bias, attitude and aiding over time from the newest PX4 ULog of a sim slot.
    SIM_ID=0 python3 sim/tools/ulog_bias.py            (needs: pip install --user pyulog)
"""
import glob, math, os, sys
from pyulog import ULog

sim = os.environ.get("SIM_ID", "0")
logs = sorted(glob.glob(os.path.expanduser(f"~/.px4_sim/{sim}/log/*/*.ulg")), key=os.path.getmtime)
if not logs:
    sys.exit("no ulog")
u = ULog(logs[-1])
print("log:", logs[-1])
d = {m.name: m for m in u.data_list}

def series(topic, field):
    m = d.get(topic)
    if m is None or field not in m.data:
        return None
    return m.data["timestamp"], m.data[field]

def at(ts, vals, t):
    i = min(range(len(ts)), key=lambda k: abs(ts[k] - t)) if len(ts) else None
    return None if i is None else vals[i]

bx = series("estimator_sensor_bias", "accel_bias[0]")
by = series("estimator_sensor_bias", "accel_bias[1]")
att = d.get("vehicle_attitude")
ev = d.get("vehicle_visual_odometry")
flags = d.get("estimator_status_flags")
t0 = u.start_timestamp
end = max(bx[0]) if bx else t0
print(" t(s)  bias_x  bias_y   roll  pitch   yaw | ev_x   ev_y   ev_yaw | ev_pos ev_yaw rng_hgt baro_hgt")
step = 2_000_000
t = t0
while t <= end:
    row = []
    row.append(f"{(t - t0) / 1e6:5.1f}")
    row.append(f"{at(*bx, t):7.3f} {at(*by, t):7.3f}" if bx else "   -       -   ")
    if att is not None:
        qs = [att.data[f"q[{i}]"] for i in range(4)]
        k = min(range(len(att.data["timestamp"])), key=lambda j: abs(att.data["timestamp"][j] - t))
        w, x, y, z = (q[k] for q in qs)
        roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        row.append(f"{roll:6.2f} {pitch:6.2f} {yaw:6.1f}")
    if ev is not None and len(ev.data["timestamp"]):
        k = min(range(len(ev.data["timestamp"])), key=lambda j: abs(ev.data["timestamp"][j] - t))
        qs = [ev.data[f"q[{i}]"][k] for i in range(4)]
        w, x, y, z = qs
        eyaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        row.append(f"| {ev.data['position[0]'][k]:6.3f} {ev.data['position[1]'][k]:6.3f} {eyaw:6.1f}")
    else:
        row.append("| (no EV)")
    if flags is not None:
        k = min(range(len(flags.data["timestamp"])), key=lambda j: abs(flags.data["timestamp"][j] - t))
        row.append("| " + " ".join(str(int(flags.data[f][k])) for f in ("cs_ev_pos", "cs_ev_yaw", "cs_rng_hgt", "cs_baro_hgt")))
    print(" ".join(row))
    t += step

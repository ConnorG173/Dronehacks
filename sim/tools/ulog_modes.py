#!/usr/bin/env python3
"""Print PX4 nav_state/arming changes and the failsafe flags that flip, from the newest ULog of a slot.
    SIM_ID=1 python3 sim/tools/ulog_modes.py
"""
import glob, os, sys
from pyulog import ULog

sim = os.environ.get("SIM_ID", "0")
logs = sorted(glob.glob(os.path.expanduser(f"~/.px4_sim/{sim}/log/*/*.ulg")), key=os.path.getmtime)
if not logs:
    sys.exit("no ulog")
u = ULog(logs[-1])
print("log:", logs[-1])
d = {m.name: m for m in u.data_list}
t0 = u.start_timestamp
events = []
vs = d.get("vehicle_status")
if vs is not None:
    prev = None
    for i, t in enumerate(vs.data["timestamp"]):
        cur = (int(vs.data["nav_state"][i]), int(vs.data["arming_state"][i]), int(vs.data["failsafe"][i]))
        if cur != prev:
            events.append((t, f"vehicle_status nav_state={cur[0]} arming_state={cur[1]} failsafe={cur[2]}"))
            prev = cur
ff = d.get("failsafe_flags")
if ff is not None:
    keys = [k for k in ff.data if k != "timestamp" and ff.data[k].dtype.kind in "bui"]
    prev = {}
    for i, t in enumerate(ff.data["timestamp"]):
        for k in keys:
            v = int(ff.data[k][i])
            if k in prev and v != prev[k]:
                events.append((t, f"failsafe_flags.{k} -> {v}"))
            prev[k] = v
for m in u.logged_messages:
    if any(w in m.message.lower() for w in ("offboard", "failsafe", "arm", "land", "override", "takeoff")):
        events.append((m.timestamp, f"LOG: {m.message}"))
for t, e in sorted(events):
    print(f"{(t - t0) / 1e6:8.2f}s  {e}")

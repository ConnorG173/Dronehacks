#!/usr/bin/env python3
"""Debug a localization bag: matcher outcomes, fuser status timeline, VIO steps around jumps.

    python3 tools/loc_dev/bag_loc_debug.py <bag dir>
"""
import collections
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from loc_eval import odom_track, read_bag  # noqa: E402

bag = sys.argv[1]
m = read_bag(bag, ["/team/loc/matcher_status", "/team/loc/status", "/visual_slam/tracking/odometry",
                   "/visual_slam/status", "/sim/ground_truth/odom", "/team/loc/fix"])

ms = [json.loads(x.data) for _, x in m["/team/loc/matcher_status"]]
print(f"matcher status msgs: {len(ms)}, ok: {sum(1 for s in ms if s.get('ok'))}, fixes in bag: {len(m['/team/loc/fix'])}")
print("matcher reasons:", collections.Counter(s.get("reason", "?").split(" (")[0] for s in ms).most_common(8))
keys = ("n_wall_pts", "n_free_pts", "depth_fill", "ms_depth", "ms_bev", "ms_match", "score", "n_in_cage", "image_age_s", "agl")
for k in keys:
    v = [s[k] for s in ms if isinstance(s.get(k), (int, float))]
    if v:
        v.sort()
        print(f"  {k:12s} n={len(v):4d} p50={v[len(v)//2]:.3f} p95={v[int(0.95*len(v))]:.3f}")
ex = [s for s in ms if "pose" in s][:3] + [s for s in ms if "pose" in s][-3:]
for s in ex:
    print("  example:", {k: s.get(k) for k in ("ok", "reason", "pose", "prior", "sd", "score", "n_in_cage")})

lvl = lambda v: v[0] if isinstance(v, (bytes, bytearray)) else int(v)
prev = None
t0 = m["/team/loc/status"][0][0] if m["/team/loc/status"] else 0
print("\nstatus changes:")
for t, s in m["/team/loc/status"]:
    key = (lvl(s.level), s.message)
    if key != prev:
        print(f"  t={t - t0:7.2f}  level {key[0]}  {key[1]}")
        prev = key

vio = odom_track(m["/visual_slam/tracking/odometry"])
print("\nVIO steps > 0.1 m between consecutive messages:")
import numpy as np  # noqa: E402
d = np.linalg.norm(np.diff(vio.p, axis=0), axis=1)
dt = np.diff(vio.t)
for i in np.nonzero(d > 0.1)[0][:10]:
    print(f"  t={vio.t[i+1] - vio.t[0]:7.2f}  step {d[i]:.3f} m in {dt[i]*1000:.0f} ms  "
          f"yaw step {math.degrees(abs((vio.yaw[i+1]-vio.yaw[i]+math.pi)%(2*math.pi)-math.pi)):.1f} deg")
big_dt = np.nonzero(dt > 0.2)[0]
print(f"VIO gaps > 0.2 s: {len(big_dt)}", [f"{dt[i]:.2f}s@{vio.t[i]-vio.t[0]:.1f}" for i in big_dt[:8]])
vs = [lvl(x.vo_state) if isinstance(x.vo_state, (bytes, bytearray)) else x.vo_state for _, x in m["/visual_slam/status"]]
print("vo_state counts:", collections.Counter(vs))
gt = m["/sim/ground_truth/odom"]
if gt and vio.t.size:
    g = odom_track(gt)
    print(f"stamps: VIO {vio.t[0]:.2f}..{vio.t[-1]:.2f}, GT {g.t[0]:.2f}..{g.t[-1]:.2f}")

# Matcher attempts vs ground truth at the image stamp: is the prior right, is the scan consistent?
if gt and any("stamp" in s for s in ms):
    g = odom_track(gt)
    rows = []
    for s in ms:
        if "stamp" not in s or "pose" not in s:
            continue
        p, yaw = g.interp([s["stamp"]])
        if not np.isfinite(p[0, 0]):
            continue
        pr, po = np.array(s["prior"]), np.array(s["pose"])
        ep = math.hypot(pr[0] - p[0, 0], pr[1] - p[0, 1])
        em = math.hypot(po[0] - p[0, 0], po[1] - p[0, 1])
        eyp = math.degrees((pr[2] - yaw[0] + math.pi) % (2 * math.pi) - math.pi)
        eym = math.degrees((po[2] - yaw[0] + math.pi) % (2 * math.pi) - math.pi)
        rows.append((s["stamp"], ep, em, eyp, eym, s.get("score_at_prior", float("nan")), s.get("score", float("nan")), s.get("ok")))
    R = np.array([r[:7] for r in rows], float)
    if len(R):
        print(f"\nattempts with GT: {len(R)}")
        for name, col in (("prior err xy", 1), ("match err xy", 2), ("prior err yaw deg", 3), ("match err yaw deg", 4),
                          ("score at prior", 5), ("score at match", 6)):
            v = np.abs(R[:, col]) if "err" in name else R[:, col]
            print(f"  {name:18s} p10 {np.nanpercentile(v,10):.3f}  p50 {np.nanpercentile(v,50):.3f}  p90 {np.nanpercentile(v,90):.3f}")
        good = R[R[:, 1] < 0.05]
        if len(good):
            print(f"  attempts with prior within 5 cm of GT: {len(good)}, their score at prior p50 {np.nanmedian(good[:,5]):.3f}")

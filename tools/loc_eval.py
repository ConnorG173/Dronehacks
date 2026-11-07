#!/usr/bin/env python3
"""Localization evaluation against sim ground truth, from a rosbag2 bag (milestone 2).

    python3 tools/loc_eval.py BAG --config planner/config/course_v3.yaml [--out DIR] [--label NAME]

Needs rosbag2_py (any ROS 2 Humble shell: the sim VM, the stack container, the dev container).
Writes DIR/summary.json, DIR/summary.md (one results-table row + details) and DIR/series.csv.
Plot on a laptop with `--plot` (matplotlib), reading series.csv.

Estimates compared, all in the course frame and without any fitted alignment (absolute error is
what flies):
  odom_map   /team/odom_map, what localization published
  vio_gt0    raw VIO anchored with GROUND TRUTH at the first sample: isolates VIO drift from
             anchor error (an oracle baseline; never available in flight)

Metrics: ATE RMS / max (horizontal, vertical, yaw) over the airborne part; error at each
checkpoint's closest approach (in order), at the goal, at the end; VIO drift per metre; latency of
/team/odom_map behind the VIO message with the same stamp (bag receive times); status honesty.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

T_GT = "/sim/ground_truth/odom"
T_VIO = "/visual_slam/tracking/odometry"
T_OUT = "/team/odom_map"
T_STATUS = "/team/loc/status"
T_VO = "/visual_slam/status"


# ----------------------------------------------------------------------------------- pure core
def quat_yaw(qx, qy, qz, qw):
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


class Track:
    """Time series of poses: t (s), p (N,3), q (N,4 xyzw), optional recv (N,) receive times."""

    def __init__(self, t, p, q, recv=None):
        o = np.argsort(t)
        self.t = np.asarray(t, float)[o]
        self.p = np.asarray(p, float)[o]
        self.q = np.asarray(q, float)[o]
        self.recv = None if recv is None else np.asarray(recv, float)[o]
        self.yaw = quat_yaw(*self.q.T) if len(self.t) else np.zeros(0)

    def __len__(self):
        return len(self.t)

    def interp(self, t):
        """Position and yaw at times t (linear; yaw unwrapped). NaN outside the track."""
        t = np.asarray(t, float)
        p = np.stack([np.interp(t, self.t, self.p[:, i], left=np.nan, right=np.nan) for i in range(3)], 1)
        yaw = wrap(np.interp(t, self.t, np.unwrap(self.yaw), left=np.nan, right=np.nan))
        return p, yaw


def anchor_with_gt(vio: Track, gt: Track) -> Track:
    """Raw VIO mapped into the course frame by matching ground truth at the first VIO sample."""
    t0 = vio.t[0]
    i = int(np.searchsorted(gt.t, t0))
    i = min(max(i, 0), len(gt) - 1)
    R_gt, R_v = quat_to_R(gt.q[i]), quat_to_R(vio.q[0])
    p_gt, _ = gt.interp([t0])
    R = R_gt @ R_v.T
    tr = p_gt[0] - R @ vio.p[0]
    p = (R @ vio.p.T).T + tr
    dyaw = math.atan2(R[1, 0], R[0, 0])
    yaw = wrap(vio.yaw + dyaw)
    q = np.stack([np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2), np.cos(yaw / 2)], 1)
    return Track(vio.t, p, q, vio.recv)


def errors(est: Track, gt: Track):
    """Per-sample errors of est vs gt at est's stamps: horizontal, vertical, yaw (rad)."""
    p, yaw = gt.interp(est.t)
    ok = np.isfinite(p[:, 0])
    eh = np.hypot(est.p[:, 0] - p[:, 0], est.p[:, 1] - p[:, 1])
    ev = est.p[:, 2] - p[:, 2]
    ey = wrap(est.yaw - yaw)
    return ok, eh, ev, ey


def airborne_mask(gt: Track, t, z_above=0.25):
    """True where ground-truth height is more than z_above above its starting height."""
    z0 = gt.p[0, 2]
    z = np.interp(t, gt.t, gt.p[:, 2])
    return z > z0 + z_above


def path_length(gt: Track, t0, t1):
    m = (gt.t >= t0) & (gt.t <= t1)
    p = gt.p[m]
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))) if len(p) > 1 else 0.0


def checkpoint_times(gt: Track, centers):
    """Time of closest approach to each centre, in order (each search starts after the previous)."""
    out, i0 = [], 0
    for c in centers:
        if i0 >= len(gt):
            out.append((math.nan, math.nan))
            continue
        d = np.linalg.norm(gt.p[i0:] - np.asarray(c), axis=1)
        k = int(np.argmin(d))
        out.append((float(gt.t[i0 + k]), float(d[k])))
        i0 += k
    return out


def latency_ms(est: Track, vio: Track):
    """Receive-time lag of est behind the VIO message with the same header stamp (ms)."""
    if est.recv is None or vio.recv is None:
        return np.zeros(0)
    idx = {round(t, 6): r for t, r in zip(vio.t, vio.recv)}
    lag = [(r - idx[round(t, 6)]) * 1000 for t, r in zip(est.t, est.recv) if round(t, 6) in idx]
    return np.asarray(lag)


def status_honesty(status_t, status_level, err_t, err_h, threshold=0.30):
    """Fraction of airborne error samples above `threshold` while the latest status said OK, and
    the lead time from the first WARN/ERROR to the first threshold crossing (s, + = warned early)."""
    if len(status_t) == 0 or len(err_t) == 0:
        return {"bad_ok_frac": math.nan, "lead_s": math.nan}
    o = np.argsort(status_t)
    st, sl = np.asarray(status_t)[o], np.asarray(status_level)[o]
    k = np.searchsorted(st, err_t, side="right") - 1
    lvl = np.where(k >= 0, sl[np.clip(k, 0, None)], 2)
    over = err_h > threshold
    bad_ok = float(np.mean(over & (lvl == 0))) if len(over) else math.nan
    lead = math.nan  # no crossing at all
    if over.any():
        t_cross = float(err_t[np.argmax(over)])
        k = int(np.searchsorted(st, t_cross, side="right")) - 1
        if k >= 0 and sl[k] >= 1:  # degraded at the crossing: how long before did that episode start?
            j = k
            while j > 0 and sl[j - 1] >= 1:
                j -= 1
            lead = t_cross - float(st[j])
        else:  # OK at the crossing: negative = how late the first WARN came (or -inf if never)
            later = st[(st > t_cross) & (sl >= 1)]
            lead = -(float(later[0]) - t_cross) if len(later) else -math.inf
    return {"bad_ok_frac": bad_ok, "lead_s": lead}


def summarize(name, est: Track, gt: Track, cps, goal, vio: Track | None):
    ok, eh, ev, ey = errors(est, gt)
    air = ok & airborne_mask(gt, est.t)
    s = {"name": name, "n": int(ok.sum()), "n_air": int(air.sum())}
    if air.any():
        s.update(ate_h_rms=float(np.sqrt(np.mean(eh[air] ** 2))), ate_h_max=float(eh[air].max()),
                 ate_v_rms=float(np.sqrt(np.mean(ev[air] ** 2))),
                 yaw_rms_deg=float(np.degrees(np.sqrt(np.mean(ey[air] ** 2)))))
    cp_err = []
    for (tc, dist) in cps:
        if math.isnan(tc):
            cp_err.append(None)
            continue
        j = int(np.argmin(np.abs(est.t - tc)))
        cp_err.append({"t": tc, "gt_dist_to_centre": dist, "err_h": float(eh[j]), "err_v": float(ev[j])})
    s["checkpoints"] = cp_err
    if goal is not None:
        (tg, dg), = checkpoint_times(gt, [goal])
        j = int(np.argmin(np.abs(est.t - tg)))
        s["goal"] = {"t": tg, "gt_dist": dg, "err_h": float(eh[j]), "err_v": float(ev[j])}
    if ok.any():
        j = np.where(ok)[0][-1]
        s["end"] = {"t": float(est.t[j]), "err_h": float(eh[j]), "err_v": float(ev[j])}
        L = path_length(gt, est.t[ok][0], est.t[j])
        s["path_m"] = L
        s["end_err_per_m"] = float(eh[j] / L) if L > 0.5 else math.nan
    if vio is not None:
        lat = latency_ms(est, vio)
        if len(lat):
            s["latency_ms"] = {"p50": float(np.percentile(lat, 50)), "p95": float(np.percentile(lat, 95)),
                               "max": float(lat.max()), "n": int(len(lat))}
    return s, (est.t, eh, ev, ey, air)


def read_course(path):
    """Checkpoint centres, Goal and StartPos from the planner's flat config."""
    cps, goal, start = [], None, None
    for line in open(path):
        line = line.split("#")[0].strip()
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        nums = v.replace("[", " ").replace("]", " ").replace(",", " ").split()
        try:
            nums = [float(x) for x in nums]
        except ValueError:
            continue
        if k.strip() == "Checkpoint":
            cps.append(nums[:3])
        elif k.strip() == "Goal":
            goal = nums[:3]
        elif k.strip() == "StartPos":
            start = nums[:3]
    return cps, goal, start


# ----------------------------------------------------------------------------------- bag I/O
def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage = "mcap" if path.endswith(".mcap") or any(f.endswith(".mcap") for f in os.listdir(path)) else "sqlite3"
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
           rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    out = {t: [] for t in topics}
    cls = {t: get_message(types[t]) for t in topics if t in types}
    while r.has_next():
        topic, data, t_ns = r.read_next()
        out[topic].append((t_ns * 1e-9, deserialize_message(data, cls[topic])))
    return out


def odom_track(msgs):
    t, p, q, recv = [], [], [], []
    for tr, m in msgs:
        h = m.header.stamp
        t.append(h.sec + h.nanosec * 1e-9)
        pp, qq = m.pose.pose.position, m.pose.pose.orientation
        p.append([pp.x, pp.y, pp.z])
        q.append([qq.x, qq.y, qq.z, qq.w])
        recv.append(tr)
    return Track(t, np.array(p).reshape(-1, 3), np.array(q).reshape(-1, 4), recv)


def md_row(label, s_out, s_raw):
    def f(d, k, scale=1.0, fmt="{:.3f}"):
        v = d.get(k) if d else None
        return "–" if v is None or (isinstance(v, float) and math.isnan(v)) else fmt.format(v * scale)
    g = lambda d, k: f(d.get("goal", {}) if d else {}, k)
    lat = s_out.get("latency_ms", {}) if s_out else {}
    return (f"| {label} | {f(s_out,'ate_h_rms')} | {f(s_out,'ate_h_max')} | {g(s_out,'err_h')} | "
            f"{f(s_out.get('end',{}) if s_out else {}, 'err_h')} | {f(s_out,'yaw_rms_deg', fmt='{:.2f}')} | "
            f"{f(s_raw,'ate_h_rms')} | {g(s_raw,'err_h')} | {f(s_raw,'end_err_per_m', 100, '{:.2f}')} % | "
            f"{f(lat,'p95', fmt='{:.2f}')} |")


MD_HEADER = ("| run | ATE h RMS (m) | ATE h max | goal err h | end err h | yaw RMS (°) | raw VIO ATE h RMS | "
             "raw goal err h | raw drift/m | odom_map lag p95 (ms) |\n|" + "---|" * 10)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--label", default=None)
    ap.add_argument("--plot", action="store_true", help="only plot DIR/series.csv (needs matplotlib)")
    a = ap.parse_args()
    out = a.out or (a.bag.rstrip("/") + "_eval")
    if a.plot:
        return plot(out)
    os.makedirs(out, exist_ok=True)
    cps, goal, _ = read_course(a.config)
    msgs = read_bag(a.bag, [T_GT, T_VIO, T_OUT, T_STATUS, T_VO])
    if not msgs[T_GT]:
        sys.exit(f"no {T_GT} in the bag: evaluation needs sim ground truth")
    gt = odom_track(msgs[T_GT])
    vio = odom_track(msgs[T_VIO]) if msgs[T_VIO] else None
    summary = {"bag": os.path.abspath(a.bag), "label": a.label or os.path.basename(a.bag.rstrip("/"))}
    cp_t = checkpoint_times(gt, cps)
    series = {}
    if msgs[T_OUT]:
        est = odom_track(msgs[T_OUT])
        summary["odom_map"], series["odom_map"] = summarize("odom_map", est, gt, cp_t, goal, vio)
        st = [(tr, m.level) for tr, m in msgs[T_STATUS]]
        if st:
            # Status is stamped by receive time; map est stamps to receive times via est.recv.
            _, eh, _, _, air = series["odom_map"]
            summary["odom_map"]["status"] = status_honesty(
                [x[0] for x in st], [x[1] for x in st], est.recv[air], eh[air])
            lv = np.array([x[1] for x in st])
            summary["odom_map"]["status_frac"] = {k: float(np.mean(lv == i)) for i, k in enumerate(["ok", "warn", "error"])}
    if vio is not None and len(vio):
        raw = anchor_with_gt(vio, gt)
        summary["vio_gt0"], series["vio_gt0"] = summarize("vio_gt0", raw, gt, cp_t, goal, None)
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1, default=float)
    with open(os.path.join(out, "series.csv"), "w") as f:
        f.write("source,t,err_h,err_v,err_yaw_deg,airborne\n")
        for k, (t, eh, ev, ey, air) in series.items():
            for row in zip(t, eh, ev, np.degrees(ey), air):
                f.write(f"{k},{row[0]:.4f},{row[1]:.4f},{row[2]:.4f},{row[3]:.3f},{int(row[4])}\n")
    row = md_row(summary["label"], summary.get("odom_map"), summary.get("vio_gt0"))
    with open(os.path.join(out, "summary.md"), "w") as f:
        f.write(MD_HEADER + "\n" + row + "\n\n```json\n" + json.dumps(summary, indent=1, default=float) + "\n```\n")
    print(MD_HEADER)
    print(row)
    for k in ("odom_map", "vio_gt0"):
        s = summary.get(k)
        if not s:
            continue
        print(f"\n{k}: checkpoints (err_h m): " +
              ", ".join("–" if c is None else f"{c['err_h']:.3f}" for c in s["checkpoints"]))
        if "status" in s:
            print(f"{k}: status {s.get('status_frac')} honesty {s['status']}")
    print(f"\nwrote {out}/summary.json, summary.md, series.csv")


def plot(out):
    import matplotlib.pyplot as plt
    rows = np.genfromtxt(os.path.join(out, "series.csv"), delimiter=",", names=True, dtype=None, encoding=None)
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    for src in sorted(set(rows["source"])):
        r = rows[rows["source"] == src]
        ax[0].plot(r["t"] - rows["t"].min(), r["err_h"], label=src)
        ax[1].plot(r["t"] - rows["t"].min(), r["err_v"], label=src)
        ax[2].plot(r["t"] - rows["t"].min(), r["err_yaw_deg"], label=src)
    for a_, y in zip(ax, ["horizontal error (m)", "vertical error (m)", "yaw error (deg)"]):
        a_.set_ylabel(y)
        a_.grid(alpha=0.3)
    ax[0].legend()
    ax[2].set_xlabel("t (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "errors.png"), dpi=120)
    print(f"wrote {out}/errors.png")


if __name__ == "__main__":
    main()

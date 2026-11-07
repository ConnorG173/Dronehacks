#!/usr/bin/env python3
"""Markdown rows from closed_loop.py summaries: python3 tools/loc_dev/cl_table.py DIR [DIR ...]"""
import json
import os
import sys

print("| scenario | path m | corrected RMS / max / end (m) | raw (M1) RMS / max / end (m) | yaw RMS corr / raw (°) "
      "| fixes acc / rej / reacq | status OK / WARN / ERROR (air, 10 Hz) | err > 0.3 m while OK | err > 3σ |")
print("|" + "---|" * 9)
for d in sys.argv[1:]:
    s = json.load(open(os.path.join(d, "summary.json")))
    c, r, f, st = s["corrected"], s["raw_anchor_only"], s["fixes"], s["status_levels_air"]
    sc = s["scenario"]
    name = (f"{os.path.basename(d.rstrip('/'))}: yaw {sc['yaw_drift_deg_per_m']}°/m, scale {sc['scale']}, "
            f"speed ×{sc['time_scale']}" + (f", dropout {sc['dropout']}" if sc["dropout"] else "")
            + ("" if sc["correction"] else ", correction off"))
    print(f"| {name} | {s['path_m']:.1f} | {c['rms']:.3f} / {c['max']:.3f} / {c['end']:.3f} | "
          f"{r['rms']:.3f} / {r['max']:.3f} / {r['end']:.3f} | {c['yaw_rms_deg']:.1f} / {r['yaw_rms_deg']:.1f} | "
          f"{f['fixes_accepted']} / {f['fixes_rejected']} / {f['reacquired']} | {st['0']} / {st['1']} / {st['2']} | "
          f"{100 * s['honesty_bad_ok']:.0f} % | {100 * s['sigma_vs_err_frac_over_3sig']:.0f} % |")

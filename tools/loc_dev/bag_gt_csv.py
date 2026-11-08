#!/usr/bin/env python3
"""Ground truth from a sim bag as CSV rows t,x,y,yaw_rad (for plot_scan.py --gt-csv).
    python3 tools/loc_dev/bag_gt_csv.py <bag> <out.csv>"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from loc_eval import odom_track, read_bag  # noqa: E402

g = odom_track(read_bag(sys.argv[1], ["/sim/ground_truth/odom"])["/sim/ground_truth/odom"])
np.savetxt(sys.argv[2], np.stack([g.t, g.p[:, 0], g.p[:, 1], g.yaw], 1), delimiter=",", fmt="%.5f")
print(f"{len(g.t)} rows -> {sys.argv[2]}")

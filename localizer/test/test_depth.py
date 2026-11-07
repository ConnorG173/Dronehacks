"""depth.Stereo on a synthetic pair: a random texture on a fronto-parallel plane. Needs cv2
(skipped on the laptop; runs in skybeta-stack, which has cv2 4.5.4)."""
import os
import sys

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
sys.path[:0] = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")]
from localizer.depth import Stereo  # noqa: E402


def test_stereo_plane_depth():
    w, h, f, b, z = 1024, 640, 682.2, 0.0756, 2.0
    K = [f, 0, w / 2, 0, f, h / 2, 0, 0, 1]
    st = Stereo(K, [], K, [], (w, h), np.eye(3), [b, 0, 0], scale=0.5, num_disp=96, block=5)
    assert st.baseline == pytest.approx(b, rel=1e-6)
    d = int(round(f * b / z))                                   # 26 px at full size
    rng = np.random.default_rng(0)
    tex = cv2.resize(rng.integers(0, 255, (h // 4, (w + d) // 4), dtype=np.uint8), (w + d, h),
                     interpolation=cv2.INTER_NEAREST)
    left, right = tex[:, :w].copy(), tex[:, d:d + w].copy()    # a point at left u is at right u - d
    _, depth = st.depth(left, right)
    zz = depth[40:-40, 120:-40]
    assert np.isfinite(zz).mean() > 0.8
    assert np.nanmedian(zz) == pytest.approx(f * b / d, rel=0.03)

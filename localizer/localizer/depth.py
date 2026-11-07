"""Depth sources for course_matcher, moved from the prototype (~/team_ws/claude/bev_live.py).

  Stereo         rectification + OpenCV SGBM at reduced resolution (CPU). The default source:
                 on the bench it kept 5 Hz next to cuVSLAM (36 ms p50 at 512x320).
  DepthAnything  DA-V2-S relative TensorRT engine (GPU), metric via ScaleFit against stereo.
                 Optional (A/B on course bags); with it the bench reached only 3.3 Hz.
  ScaleFit       1/z = s * rel + t, robust fit to stereo pixels. Only for the RELATIVE engine:
                 the metric-indoor engine outputs depth, not inverse depth, and fails this fit.

cv2 / tensorrt / torch are imported lazily so the pure parts load anywhere (laptop tests).
Subscribe to /oak/*/image_mono (mono8): a third of the bytes of image_raw (bgr8), which is the
likely cause of the bench's rate shortfall.
"""
from __future__ import annotations

import numpy as np


def img_to_np(m):
    """sensor_msgs/Image (mono8, 8UC1, bgr8, rgb8) -> uint8 grey (h, w)."""
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)
    if m.encoding in ("mono8", "8UC1"):
        return a[:, :m.width].copy()
    if m.encoding in ("bgr8", "rgb8"):
        import cv2
        a = a[:, :m.width * 3].reshape(m.height, m.width, 3)
        return cv2.cvtColor(a, cv2.COLOR_BGR2GRAY if m.encoding == "bgr8" else cv2.COLOR_RGB2GRAY)
    raise ValueError(f"unsupported encoding {m.encoding}")


class Stereo:
    """Rectification maps + SGBM at a reduced resolution.
    K1/D1, K2/D2: left/right intrinsics (CameraInfo k, d) at full size (w, h). R_lr, t_lr: the
    right camera's pose in the left optical frame (X_l = R_lr X_r + t_lr), from TF.
    After construction: size (w, h), f, cx, cy, baseline, and R1 (left optical -> rectified), so
    R_body_rect = R_body_leftoptical @ R1.T for frame_bev."""

    def __init__(self, K1, D1, K2, D2, size_wh, R_lr, t_lr, scale=0.5, num_disp=96, block=5):
        import cv2
        w, h = size_wh
        self.size = (int(round(w * scale)), int(round(h * scale)))
        K1, K2 = np.asarray(K1, float).reshape(3, 3), np.asarray(K2, float).reshape(3, 3)
        D1 = np.asarray(D1, float) if len(D1) else np.zeros(5)
        D2 = np.asarray(D2, float) if len(D2) else np.zeros(5)
        R_lr, t_lr = np.asarray(R_lr, float), np.asarray(t_lr, float)
        R = R_lr.T                                        # OpenCV wants X_r = R X_l + T
        T = -R_lr.T @ t_lr
        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(K1, D1, K2, D2, (w, h), R, T,
                                                    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
                                                    newImageSize=self.size)
        self.mapL = cv2.initUndistortRectifyMap(K1, D1, R1, P1, self.size, cv2.CV_16SC2)
        self.mapR = cv2.initUndistortRectifyMap(K2, D2, R2, P2, self.size, cv2.CV_16SC2)
        self.R1 = R1
        self.f, self.cx, self.cy = float(P1[0, 0]), float(P1[0, 2]), float(P1[1, 2])
        self.baseline = float(abs(P2[0, 3] / P2[0, 0]))
        self.num_disp = num_disp
        self.sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=num_disp, blockSize=block,
                                          P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
                                          uniquenessRatio=15, speckleWindowSize=80, speckleRange=2,
                                          preFilterCap=31, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    @property
    def K(self):
        return (self.f, self.f, self.cx, self.cy)

    def depth(self, grayL, grayR):
        """Full-size grey pair -> (rectified left, depth metres with NaN = no match)."""
        import cv2
        l = cv2.remap(grayL, *self.mapL, cv2.INTER_LINEAR)
        r = cv2.remap(grayR, *self.mapR, cv2.INTER_LINEAR)
        disp = self.sgbm.compute(l, r).astype(np.float32) / 16.0
        z = np.full(disp.shape, np.nan, np.float32)
        ok = disp > 0.5
        z[ok] = self.f * self.baseline / disp[ok]
        z[:, :self.num_disp] = np.nan                     # SGBM has no match support there
        return l, z


class DepthAnything:
    """Depth Anything V2 Small (relative) TensorRT FP16 engine on torch CUDA buffers.
    Output: relative inverse depth (larger = nearer) at the engine's input size (w, h)."""

    def __init__(self, engine_path, h=252, w=406):
        import tensorrt as trt
        import torch
        self.torch = torch
        log = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(log).deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        self.h, self.w = h, w
        self.ctx.set_input_shape("pixel_values", (1, 3, h, w))
        out_shape = tuple(self.ctx.get_tensor_shape("predicted_depth"))
        self.inp = torch.empty((1, 3, h, w), dtype=torch.float32, device="cuda")
        self.out = torch.empty(out_shape, dtype=torch.float32, device="cuda")
        self.ctx.set_tensor_address("pixel_values", self.inp.data_ptr())
        self.ctx.set_tensor_address("predicted_depth", self.out.data_ptr())
        self.mean = torch.tensor([0.485, 0.456, 0.406], device="cuda").view(1, 3, 1, 1)
        self.std = torch.tensor([0.229, 0.224, 0.225], device="cuda").view(1, 3, 1, 1)
        self.stream = torch.cuda.Stream()

    def __call__(self, gray):
        import cv2
        torch = self.torch
        small = cv2.resize(gray, (self.w, self.h), interpolation=cv2.INTER_AREA)
        with torch.cuda.stream(self.stream):
            t = torch.from_numpy(small).to("cuda", non_blocking=True).float().div_(255.0)
            self.inp.copy_((t.view(1, 1, self.h, self.w).expand(1, 3, self.h, self.w) - self.mean) / self.std)
            self.ctx.execute_async_v3(self.stream.cuda_stream)
            out = self.out[0].cpu()
        self.stream.synchronize()
        return out.numpy()


class ScaleFit:
    """Metric scale for RELATIVE inverse depth: 1/z = s * rel + t, fitted robustly (RANSAC on two
    parameters, then least squares on the inliers) to stereo pixels in 0.3-3.5 m, smoothed over
    frames. Stereo contributes only these two numbers; the geometry is the network's."""

    def __init__(self, alpha=0.3, seed=0):
        self.s = self.t = None
        self.alpha = alpha
        self.n_in = 0
        self.rng = np.random.default_rng(seed)

    def update(self, rel, z_st):
        ok = np.isfinite(z_st) & (z_st > 0.3) & (z_st < 3.5) & np.isfinite(rel)
        n = int(ok.sum())
        if n < 300:
            self.n_in = 0
            return False
        r, y = rel[ok].astype(np.float64), 1.0 / z_st[ok].astype(np.float64)
        if n > 4000:
            k = self.rng.choice(n, 4000, replace=False)
            r, y = r[k], y[k]
        best, best_in = None, None
        for _ in range(80):
            i, j = self.rng.integers(0, len(r), 2)
            if abs(r[i] - r[j]) < 1e-6:
                continue
            s = (y[i] - y[j]) / (r[i] - r[j])
            if s <= 0:
                continue
            t = y[i] - s * r[i]
            inl = np.abs(y - (s * r + t)) < 0.1 * y
            if best_in is None or inl.sum() > best_in.sum():
                best, best_in = (s, t), inl
        if best is None or best_in.sum() < 200:
            self.n_in = 0
            return False
        A = np.stack([r[best_in], np.ones(int(best_in.sum()))], 1)
        s, t = np.linalg.lstsq(A, y[best_in], rcond=None)[0]
        if s <= 0:
            return False
        self.n_in = int(best_in.sum())
        if self.s is None:
            self.s, self.t = s, t
        else:
            self.s += self.alpha * (s - self.s)
            self.t += self.alpha * (t - self.t)
        return True

    def metric(self, rel, zmax=8.0):
        if self.s is None:
            return None
        inv = self.s * rel + self.t
        z = np.full(rel.shape, np.nan, np.float32)
        ok = inv > 1.0 / zmax
        z[ok] = 1.0 / inv[ok]
        return z

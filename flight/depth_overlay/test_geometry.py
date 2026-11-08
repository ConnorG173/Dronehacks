"""CPU-only synthetic projection checks; no ROS, camera or GPU access."""
import math
import unittest

import numpy as np

from geometry import (NED_TO_ENU, _rotation_from_px4, nearest_sample,
                      project_depth, scaled_intrinsics)


R_BODY_CAM = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
T_BODY_CAM = np.array([.1, .02, .03])
W, H, F = 160, 120, 100.
K = (F, F, W/2, H/2)


def quaternion(roll=0., pitch=0., yaw=0.):
    r, p, y = np.radians([roll, pitch, yaw])/2
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def wall_depth(q, forward_distance=2., floor_relative=None):
    """Render vertical wall perpendicular to level body-forward, optionally a floor."""
    r = _rotation_from_px4(q)
    yaw = math.atan2(r[1, 0], r[0, 0])
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.])
    v, u = np.mgrid[:H, :W]
    rays = np.stack([(u-K[2])/F, (v-K[3])/F, np.ones((H, W))], axis=-1)
    dirs = rays @ (r @ R_BODY_CAM).T
    origin = r @ T_BODY_CAM
    with np.errstate(divide='ignore', invalid='ignore'):
        z = (forward_distance-origin @ forward)/(dirs @ forward)
        if floor_relative is not None:
            floor_z = (floor_relative-origin[2])/dirs[..., 2]
            z = np.minimum(np.where(z > 0, z, np.inf), np.where(floor_z > 0, floor_z, np.inf))
    return np.where((z > 0) & np.isfinite(z), z, np.nan)


class GeometryTests(unittest.TestCase):
    def project(self, depth, q=(1., 0., 0., 0.), position=(3., 5., -1.), **kw):
        return project_depth(depth, K, R_BODY_CAM, T_BODY_CAM, position, q, **kw)

    def test_north_heading_wall_and_camera_offset(self):
        result = self.project(wall_depth(quaternion()))
        points = np.array(result['points'])
        self.assertGreater(len(points), 20)
        self.assertTrue(np.all(abs(points[:, 1]-5.) <= .08))  # N=3, wall2m north.
        self.assertTrue(np.allclose(result['camera_xy'], [4.98, 3.1]))
        self.assertTrue(np.isfinite(points).all())

    def test_east_heading_rotates_the_wall_and_camera_offset(self):
        q = quaternion(yaw=90)
        result = self.project(wall_depth(q), q)
        points = np.array(result['points'])
        self.assertGreater(len(points), 20)
        self.assertTrue(np.all(abs(points[:, 0]-7.) <= .08))
        self.assertTrue(np.allclose(result['camera_xy'], [5.1, 3.02]))

    def test_roll_and_pitch_keep_wall_and_height_band_gravity_referenced(self):
        q = quaternion(roll=15, pitch=-12, yaw=90)
        result = self.project(wall_depth(q, floor_relative=-.2), q)
        points = np.array(result['points'])
        self.assertGreater(len(points), 10)
        self.assertTrue(np.all(abs(points[:, 0]-7.) <= .08))

    def test_flat_floor_in_height_band_is_not_a_vertical_obstacle(self):
        q = quaternion(roll=15, pitch=-12)
        depth = wall_depth(q, forward_distance=100., floor_relative=-.2)
        result = self.project(depth, q)
        self.assertEqual(result['points'], [])

    def test_window_reports_sill_and_header_as_separate_height_slabs(self):
        """2 m wall with a 0.9 m wide, 0.6 m tall opening centred on the drone (body at z=1)."""
        q = quaternion()
        depth = wall_depth(q)
        r = _rotation_from_px4(q)
        v, u = np.mgrid[:H, :W]
        rays = np.stack([(u-K[2])/F, (v-K[3])/F, np.ones((H, W))], axis=-1)
        hit = rays*depth[..., None] @ (r @ R_BODY_CAM).T + r @ T_BODY_CAM
        lateral, up = hit[..., 0], hit[..., 2]   # facing north: sideways is ENU x
        depth = np.where((abs(lateral) < .45) & (up > -.30) & (up < .30), np.nan, depth)
        result = self.project(depth)
        points, zr = np.array(result['points']), np.array(result['z_ranges'])
        self.assertEqual(len(points), len(zr))
        flying = (zr[:, 0] <= 1.25) & (zr[:, 1] >= .75)     # drone at z=1, +/-0.25 m
        inside = (abs(points[:, 0]-5.) < .45-.08) & (abs(points[:, 1]-5.) <= .08)
        self.assertTrue(inside.any(), 'sill/header should still be reported at the opening')
        self.assertFalse((inside & flying).any(), 'opening reported as blocked at flying height')
        jamb = (abs(points[:, 0]-5.) > .45+.08) & flying
        self.assertTrue((jamb & (points[:, 0] < 5.)).any() and (jamb & (points[:, 0] > 5.)).any(), 'jambs missing')

    def test_invalid_depth_samples_ignored_and_invalid_geometry_rejected(self):
        for invalid in (float('nan'), float('inf'), 0., -.2, .1, 4., 5.):
            self.assertEqual(self.project(np.full((H,W), invalid))['points'], [])
        for q in ([0.,0.,0.,0.], [float('nan'),0.,0.,0.], [2.,0.,0.,0.]):
            with self.assertRaises(ValueError): self.project(np.ones((H,W)), q)
        with self.assertRaises(ValueError):
            project_depth(np.ones((H,W)), K, np.zeros((3,3)), T_BODY_CAM, [0.,0.,0.], quaternion())
        with self.assertRaises(ValueError):
            self.project(np.ones((H,W)), position=[0.,float('nan'),0.])

    def test_cell_limit_is_bounded_and_retains_finite_fixed_grid_centres(self):
        result = self.project(wall_depth(quaternion()), max_points=7)
        points = np.array(result['points'])
        self.assertEqual(len(points),7)
        self.assertTrue(np.isfinite(points).all())
        self.assertTrue(np.allclose(points/.08-.5, np.round(points/.08-.5)))

    def test_resized_intrinsics_follow_cv_pixel_centres(self):
        self.assertEqual(scaled_intrinsics((100.,120.,79.5,59.5),(160,120),(80,30)),
                         (50.,30.,39.5,14.5))
        with self.assertRaises(ValueError): scaled_intrinsics(K,(0,120),(80,60))

    def test_nearest_pose_at_capture_time_and_bound(self):
        samples = [(1_000_000_000,'capture'),(1_100_000_000,'later'),(2_000_000_000,'latest')]
        self.assertEqual(nearest_sample(samples,1_020_000_000),'capture')
        self.assertEqual(nearest_sample(samples,1_250_000_000),'later')
        self.assertIsNone(nearest_sample(samples,1_250_000_001))
        self.assertIsNone(nearest_sample([],1))
        self.assertEqual(nearest_sample(list(reversed(samples)),1_050_000_000),'capture')


if __name__ == '__main__':
    unittest.main()



class ScaleChooserTests(unittest.TestCase):
    def frame(self, s, t, n=4000, seed=0, scale=1.):
        rng = np.random.default_rng(seed)
        z = rng.uniform(.6, 3.2, n)
        rel = ((1/z - t)/s)*scale                          # what the network would output
        return rel, z*(1+rng.normal(0, .02, n))           # 2 % stereo noise

    def test_this_frames_stereo_decides_and_an_old_scale_is_dropped_when_it_no_longer_fits(self):
        from geometry import ScaleChooser
        ch = ScaleChooser()
        rel, z = self.frame(.25, .03)
        q = np.nanpercentile(rel, [10, 50, 90])
        st, state, sup = ch.choose([(.25, .03), (.55, .35), (.03, .91)], rel, z, q)   # wrong ones fit poorly
        self.assertEqual(state, 'new'); self.assertAlmostEqual(st[0], .25, places=6); self.assertGreater(sup, .9)
        st, state, _ = ch.choose([(.252, .031)], rel, z, q)                             # same scene: blend
        self.assertEqual(state, 'blend')
        rel2, z2 = self.frame(.25, .03, seed=1, scale=1.6)   # network output magnitude changes
        q2 = np.nanpercentile(rel2, [10, 50, 90])
        st, state, sup = ch.choose([(.25/1.6, .03)], rel2, z2, q2)
        self.assertEqual(state, 'switch')                    # the held (s, t) no longer fits: dropped
        self.assertGreater(sup, .9)

    def test_first_scale_waits_for_stereo_over_a_range_of_depths(self):
        from geometry import ScaleChooser
        rng = np.random.default_rng(5)
        rel = rng.uniform(1., 5., 3000); z = np.full(3000, 1.5)             # one wall at 1.5 m
        q = np.nanpercentile(rel, [10, 50, 90])
        st, state, _ = ScaleChooser().choose([(.01, .6), (.25, .1)], rel, z, q)
        self.assertEqual((st, state), (None, 'flat'))

    def test_negative_offset_scale_is_usable_with_the_far_end_capped(self):
        from geometry import ScaleChooser, scale_contrast, FAR_CAP_M
        q = np.array([.05, .5, 1.0])
        st = (.35, -.03)                          # far end: .35*.05-.03 < 0 -> beyond range
        self.assertTrue(np.isfinite(scale_contrast(st, q)))
        self.assertAlmostEqual(scale_contrast(st, q), math.log(FAR_CAP_M*(.35-.03)), places=6)
        rel = np.linspace(.05, 4., 2000); z = 1/(.35*rel-.03); z[(z <= 0) | (z > 3.5)] = np.nan
        st2, state, sup = ScaleChooser().choose([st], rel, z, q)
        self.assertEqual(st2, st); self.assertGreater(sup, .9)

    def test_tied_support_prefers_more_contrast_and_then_stays(self):
        from geometry import ScaleChooser, scale_contrast
        rng = np.random.default_rng(3)
        rel = rng.uniform(1., 5., 4000); z = np.full(4000, np.nan); z[:200] = 1.5   # stereo only at one depth
        rel[:200] = 2.0 + rng.normal(0, .005, 200)                                  # (one ML value there too):
        q = np.nanpercentile(rel, [10, 50, 90])                                     # every line through it ties
        low = (.10, 1/1.5-.10*np.nanmedian(rel[:200])); high = (.30, 1/1.5-.30*np.nanmedian(rel[:200]))
        ch = ScaleChooser()
        self.assertEqual(ch.choose([low, high], rel, z, q)[1], 'flat')   # one depth band: no first scale
        zs = 1/(high[0]*rel+high[1])                              # a frame with depth spread fixes `high`
        st, state, _ = ch.choose([high], rel, zs, q)
        self.assertEqual((st, state), (high, 'new'))
        self.assertGreater(scale_contrast(high, q), scale_contrast(low, q))
        st, state, _ = ch.choose([low], rel, z, q)            # on a tie the flat one does not displace it
        self.assertEqual(st, high)
        st, state, _ = ch.choose([low, high], rel, z, q)
        self.assertEqual(st, high)

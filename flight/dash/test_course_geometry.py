"""Pure preview tests, using synthetic geometry only; no flight commands."""
import math
import unittest

import numpy as np

from course_geometry import bump, course_to_world, deform_preview, world_to_course


def minimum_segment_distance(points, walls):
    """Independent scalar oracle checks the whole rendered polyline."""
    best = math.inf
    for a,b in zip(np.asarray(points)[:-1,:2],np.asarray(points)[1:,:2]):
        v = b-a
        for wall in np.asarray(walls):
            t = max(0.,min(1.,float(np.dot(wall-a,v)/np.dot(v,v)))) if np.dot(v,v) else 0.
            best = min(best,float(np.linalg.norm(wall-(a+t*v))))
    return best


class CourseGeometryTests(unittest.TestCase):
    def test_frame_transform_ground_height_and_roundtrip(self):
        anchor = dict(origin_enu=[3.,-4.,.5],yaw_rad=math.pi/2)
        xyz = np.array([[1.,2.,3.],[-2.,.5,.8]])
        world = course_to_world(xyz,anchor)
        np.testing.assert_allclose(world[0],[1.,-3.,3.5])
        np.testing.assert_allclose(world_to_course(world,anchor),xyz,atol=1e-12)
        np.testing.assert_allclose(course_to_world(xyz[0],anchor),world[0])
        self.assertEqual(course_to_world(np.empty((0,3)),anchor).shape,(0,3))

    def test_unchanged_without_walls_does_not_claim_observed_space(self):
        result = deform_preview([[-1.,0.,1.],[1.,0.,1.]],[])
        self.assertEqual(result['status'],'unchanged')
        self.assertIsNone(result['min_clearance_m'])
        self.assertIn('unobserved',result['reason'])
        self.assertLessEqual(np.linalg.norm(np.diff(result['points'],axis=0),axis=1).max(),.05+1e-12)

    def test_conflict_between_clear_endpoints_is_detected_and_adjusted(self):
        walls = [[0.,0.]]
        result = deform_preview([[-1.,0.,1.],[1.,0.,1.]],walls)
        self.assertEqual(result['status'],'adjusted',result['reason'])
        self.assertGreaterEqual(minimum_segment_distance(result['points'],walls),.3+.08/math.sqrt(2)-1e-9)
        self.assertGreaterEqual(result['min_clearance_m'],.3-1e-9)

    def test_small_wall_avoids_without_changing_original_endpoints_or_altitude(self):
        original = np.array([[-1.,0.,.7],[0.,0.,.8],[1.,0.,.9]])
        saved = original.copy()
        walls = np.array([[0.,-.08],[0.,0.],[0.,.08]])
        walls_saved = walls.copy()
        result = deform_preview(original,walls)
        self.assertEqual(result['status'],'adjusted',result['reason'])
        p = np.asarray(result['points'])
        np.testing.assert_array_equal(original,saved)
        np.testing.assert_array_equal(walls,walls_saved)
        np.testing.assert_array_equal(p[[0,-1]],original[[0,-1]])
        np.testing.assert_allclose(p[:,2],.8+.1*p[:,0],atol=1e-12)
        self.assertLessEqual(abs(p[:,1]).max(),.60+1e-12)
        self.assertLessEqual(np.linalg.norm(np.diff(p,axis=0),axis=1).max(),.05+1e-12)
        self.assertTrue(np.isfinite(p).all())

    def test_narrow_gap_and_both_sides_blocked_stop_preview(self):
        for walls in ([[0.,-.25],[0.,.25]],[[0.,y] for y in np.arange(-1.,1.01,.08)]):
            result = deform_preview([[-1.,0.,1.],[1.,0.,1.]],walls)
            self.assertEqual(result['status'],'blocked')
            self.assertEqual(result['points'],[])
            self.assertLess(result['min_clearance_m'],.3)

    def test_endpoint_cannot_be_moved_away_from_wall(self):
        result = deform_preview([[-1.,0.,1.],[1.,0.,1.]],[[-1.,0.]])
        self.assertEqual(result['status'],'blocked')
        self.assertEqual(result['points'],[])

    def test_bump_has_zero_value_slope_and_curvature_at_joins(self):
        self.assertEqual(float(bump(0.)),0.)
        self.assertEqual(float(bump(1.)),0.)
        self.assertEqual(float(bump(.5)),1.)
        eps = 1e-6
        for end,sign in ((0.,1.),(1.,-1.)):
            f0,f1,f2 = (float(bump(end+sign*k*eps)) for k in range(3))
            self.assertLess(abs((f1-f0)/eps),1e-8)
            self.assertLess(abs((f2-2*f1+f0)/eps**2),.001)
        np.testing.assert_array_equal(bump([-2.,2.]),[0.,0.])

    def test_invalid_and_oversized_inputs_rejected(self):
        for points in ([[0.,0.,0.],[float('nan'),0.,0.]],
                       [[0.,0.,0.],[201.,0.,0.]],[[0.,0.,0.]]):
            with self.assertRaises(ValueError): deform_preview(points,[])
        with self.assertRaises(ValueError):
            deform_preview([[-1.,0.,1.],[1.,0.,1.]],[[float('nan'),0.]])
        with self.assertRaises(ValueError):
            course_to_world([[0.,0.,1.]],dict(origin_enu=[0.,0.,0.],yaw_rad=float('nan')))

    def test_preview_is_deterministic(self):
        args = ([[-1.,0.,1.],[1.,0.,1.]],[[0.,0.]])
        self.assertEqual(deform_preview(*args),deform_preview(*args))

    def test_work_limit_returns_no_unvalidated_route(self):
        result = deform_preview([[-1.,0.,1.],[1.,0.,1.]],[[0.,0.]],max_work_s=1e-12)
        self.assertEqual(result['status'],'blocked')
        self.assertEqual(result['points'],[])
        self.assertIsNone(result['min_clearance_m'])
        self.assertIn('work limit',result['reason'])

    def test_exact_spacing_near_sample_limit_does_not_double_resample(self):
        route = np.column_stack([np.arange(4000)*.05,np.zeros(4000),np.ones(4000)])
        result = deform_preview(route,[],max_work_s=1.)
        self.assertEqual(result['status'],'unchanged')
        self.assertEqual(len(result['points']),4000)


class FlyThroughOpeningTests(unittest.TestCase):
    def test_opening_kept_at_guard_clearance_is_flown_straight_through(self):
        from course_geometry import deform_preview
        def window(half):
            return [[round(-2+.08*i,2),3.] for i in range(51) if abs(-2+.08*i)>=half-1e-9]
        route=[[0,0,1],[0,6,1]]
        tight=window(.40)                       # posts 0.40 m from the route centre line
        self.assertEqual(deform_preview(route,tight,clearance_m=.40,max_offset_m=1.5)['status'],'blocked')
        r=deform_preview(route,tight,clearance_m=.40,max_offset_m=1.5,pass_clearance_m=.30)
        self.assertEqual(r['status'],'unchanged')
        self.assertTrue(all(abs(p[0])<1e-9 for p in r['points']))   # straight through, no detour
        narrow=window(.32)                      # below the guard clearance: still blocked
        self.assertEqual(deform_preview(route,narrow,clearance_m=.40,max_offset_m=1.5,pass_clearance_m=.30)['status'],'blocked')


if __name__ == '__main__':
    unittest.main()

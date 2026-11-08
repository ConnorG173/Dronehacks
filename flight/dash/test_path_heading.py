"""Pure heading-profile tests. Never connects to the running dashboard or drone."""
import math
import unittest

import numpy as np

from path_heading import tangent_yaws, wrap_angle, yaw_quaternion


class PathHeadingTests(unittest.TestCase):
    def test_left_and_right_corners_use_outgoing_geometry_not_diagonals(self):
        path = [[0.,0.,1.],[1.,0.,1.],[1.,1.,1.],[2.,1.,1.]]
        np.testing.assert_allclose(tangent_yaws(path),[0.,math.pi/2,0.,0.],atol=1e-12)
        mirrored = [[x,-y,z] for x,y,z in path]
        np.testing.assert_allclose(tangent_yaws(mirrored),[0.,-math.pi/2,0.,0.],atol=1e-12)

    def test_crossing_pi_unwraps_without_a_full_rotation(self):
        angles = np.radians([170.,179.,-179.,-170.])
        xy = np.vstack([[0.,0.],np.cumsum(np.column_stack([np.cos(angles),np.sin(angles)]),axis=0)])
        points = np.column_stack([xy,np.ones(len(xy))])
        yaw = tangent_yaws(points)
        np.testing.assert_allclose(np.degrees(yaw),[170.,179.,181.,190.,190.],atol=1e-10)
        self.assertLess(np.max(np.abs(np.diff(yaw))),math.radians(10))

    def test_duplicate_and_vertical_sections_hold_until_movement_resumes(self):
        path = [[0.,0.,0.],[0.,0.,1.],[1.,0.,1.],[1.,0.,2.],
                [1.,0.,3.],[1.,1.,3.],[1.,1.,3.],[1.,1.,4.]]
        np.testing.assert_allclose(tangent_yaws(path,initial_yaw=1.),
            [0.,0.,0.,0.,math.pi/2,math.pi/2,math.pi/2,math.pi/2],atol=1e-12)

    def test_stopped_start_uses_first_outgoing_and_stopped_finish_holds(self):
        path = [[0.,0.,0.],[0.,0.,1.],[0.,1.,1.],[0.,1.,0.]]
        np.testing.assert_allclose(tangent_yaws(path,initial_yaw=-1.),[math.pi/2]*4)

    def test_all_stationary_vertical_and_tiny_jitter_keep_initial(self):
        for path in ([[1.,2.,0.],[1.,2.,1.],[1.,2.,2.]],
                     [[0.,0.,0.],[.004,0.,0.],[.008,0.,0.]],
                     [[0.,0.,0.],[.003,0.,0.],[-.003,0.,0.],[.003,0.,0.]],
                     [[1.,2.,3.]]):
            self.assertEqual(tangent_yaws(path,initial_yaw=.7),[.7]*len(path))

    def test_fine_samples_accumulate_a_meaningful_direction(self):
        xy = np.column_stack([np.zeros(30),np.arange(30)*.001])
        points = np.column_stack([xy,np.ones(30)])
        np.testing.assert_allclose(tangent_yaws(points),[math.pi/2]*30,atol=1e-12)

    def test_reversal_is_defined_instead_of_averaging_to_zero(self):
        yaws = tangent_yaws([[0.,0.,1.],[1.,0.,1.],[0.,0.,1.]])
        self.assertTrue(np.isfinite(yaws).all())
        self.assertEqual(yaws[0],0.)
        self.assertAlmostEqual(abs(yaws[1]),math.pi)
        self.assertEqual(yaws[-1],yaws[-2])

    def test_adjusted_geometry_changes_heading_and_input_is_preserved(self):
        original = np.array([[0.,0.,1.],[1.,0.,1.],[2.,0.,1.]])
        adjusted = original.copy(); adjusted[1,1]=.3
        saved = adjusted.copy()
        self.assertEqual(tangent_yaws(original),[0.,0.,0.])
        yaws = tangent_yaws(adjusted)
        np.testing.assert_allclose(yaws,[math.atan2(.3,1.),math.atan2(-.3,1.),math.atan2(-.3,1.)])
        np.testing.assert_array_equal(adjusted,saved)

    def test_empty_invalid_and_quaternion_contract(self):
        self.assertEqual(tangent_yaws([]),[])
        for points in ([[0.,0.]], [[0.,0.,float('nan')]], [[0.,float('inf'),0.]]):
            with self.assertRaises(ValueError): tangent_yaws(points)
        with self.assertRaises(ValueError): tangent_yaws([[0.,0.,0.]],initial_yaw=float('nan'))
        with self.assertRaises(ValueError): wrap_angle(float('inf'))
        self.assertAlmostEqual(wrap_angle(2*math.pi+.2),.2)
        q = yaw_quaternion(math.pi/2)
        np.testing.assert_allclose(q,[0.,0.,math.sqrt(.5),math.sqrt(.5)])
        self.assertAlmostEqual(sum(v*v for v in q),1.)


if __name__ == '__main__':
    unittest.main()

"""Focused shadow-mode API checks; publishers/services are inert doubles."""
from unittest.mock import Mock, patch
import unittest
import dash_core
import test_autoflight_api_offline as api


class ShadowModeTests(unittest.TestCase):
    node = api.FlightApiTests.node

    def shadow(self):
        n=self.node();n.core.p.obstacle_mode='shadow'
        n.course.execution_path=Mock(side_effect=AssertionError('adjusted route used'))
        n.depth_overlay.snapshot=Mock(side_effect=AssertionError('depth used by launch'))
        return n

    def test_original_route_prepares_with_no_depth_or_collision_checks(self):
        for depth in (None,{},dict(status='stale',age_s=10),
                      dict(status='live',age_s=0,points=[[0.,0.]],cell_m=.08)):
            n=self.shadow()
            with patch.object(dash_core,'observations',side_effect=AssertionError('depth checked')):
                ok,message=n.core.prepare_takeoff_course(10.,n.course.original_path(),6.,n.pos,n.fc,depth)
            self.assertTrue(ok,message)
            self.assertIsNone(n.core.deferred_course)
            self.assertIsNone(n.core.queued_course.obstacle_guard)
            self.assertEqual(n.core.queued_course.source,'original')
            self.assertEqual(n.core.queued_course.points,[(0.,0.,1.),(4.,0.,1.)])
            self.assertLessEqual(n.core.queued_course.xy_speed,1.)
            n.arm_pub.publish.assert_not_called()

    def test_legacy_arm_prepares_original_course_without_sensing(self):
        n=self.shadow()
        ok,message=n.do_command(10.,'pilot',dict(op='arm'))
        self.assertTrue(ok,message)
        n.arm_pub.publish.assert_called_once()
        self.assertEqual(n.core.queued_course.source,'original')
        self.assertIsNone(n.core.queued_course.avoidance)

    def test_main_takeoff_queues_original_course(self):
        n=self.shadow()
        ok,message=n.do_command(10.,'pilot',dict(op='launch',flight='course',radius_m=6.))
        self.assertTrue(ok,message)
        n.arm_pub.publish.assert_called_once()
        self.assertIsNotNone(n.core.queued_course)
        self.assertIsNone(n.core.deferred_course)

    def test_geometry_and_non_depth_controls_still_reject_invalid_launches(self):
        changes=[lambda n:setattr(n.fc,'armed',True),
                 lambda n:setattr(n.pos,'t',1.),
                 lambda n:setattr(n.fc,'origin_z',2.),
                 lambda n:setattr(n.pos,'x',1.)]
        for change in changes:
            n=self.shadow();change(n)
            self.assertFalse(n.core.prepare_takeoff_course(10.,n.course.original_path(),6.,n.pos,n.fc)[0])
            n.arm_pub.publish.assert_not_called()

    def test_shadow_rejects_adjusted_path_and_preserves_speed_cap(self):
        n=self.shadow();path=n.course.original_path();path['source']='adjusted'
        ok,message=n.core.prepare_takeoff_course(10.,path,6.,n.pos,n.fc)
        self.assertFalse(ok);self.assertIn('original course',message)

    def test_takeoff_to_course_transition_has_no_route_wait(self):
        n=self.shadow()
        self.assertTrue(n.core.prepare_takeoff_course(10.,n.course.original_path(),6.,n.pos,n.fc)[0])
        n.fc.armed=True;n.fc.landed=False;n.core.was_offboard=True
        n.pos.z=1.;n.core.cmd=[0.,0.,1.]
        n.core._advance_launch(10.1,n.pos,n.fc)
        self.assertEqual(n.core.launch_state,'settling')
        n.core._advance_launch(10.7,n.pos,n.fc)
        self.assertEqual(n.core.launch_state,'flying')
        self.assertIsNone(n.core.queued_course)
        self.assertTrue(n.core.course_run.active)
        self.assertIsNone(n.core.course_run.obstacle_guard)


if __name__ == '__main__':
    unittest.main()

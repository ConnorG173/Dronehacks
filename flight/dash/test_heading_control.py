"""Synthetic turn and translation checks; no ROS or vehicle access."""
import math
import unittest

from dash_core import DashCore,Pos,Fc
from heading_control import (YawTurn,FacingTravel,MAX_RATE,MAX_ACCEL,
                             MAX_COMMAND_LEAD,limited_horizontal_step)
from path_heading import wrap_angle
import os as _os
from unittest.mock import patch as _patch
# These tests check the dashboard's in-flight holds, now off by default (DashParams.no_holds).
LEGACY_HOLDS = _patch.dict(_os.environ, {'DASH_LEGACY_HOLDS': '1'})


class TurnTests(unittest.TestCase):
    def test_shortest_turn_has_bounded_rate_acceleration_and_stopped_endpoints(self):
        for start,target,expected in ((179,-179,2),(-179,179,-2),(0,180,180),(30,-70,-100)):
            turn=YawTurn(math.radians(start),math.radians(target))
            initial=turn.yaw;last=initial
            while not turn.done:
                angle=turn.step(.01)
                self.assertLessEqual(abs(angle-last),MAX_RATE*.01+1e-9)
                self.assertLessEqual(abs(turn.rate),MAX_RATE+1e-9)
                self.assertLessEqual(abs(turn.accel),MAX_ACCEL+1e-9)
                last=angle
            self.assertAlmostEqual(math.degrees(turn.yaw-initial),expected)
            self.assertEqual((turn.rate,turn.accel),(0.,0.))

    def test_measured_alignment_is_required_even_after_commanded_turn_finishes(self):
        facing=FacingTravel();facing.reset(0.)
        for _ in range(80):
            self.assertFalse(facing.advance_allowed(0,1,0.,0.,0.,.1))
        self.assertAlmostEqual(facing.yaw,math.pi/2)
        self.assertEqual(facing.phase,'turning')
        self.assertFalse(facing.advance_allowed(0,1,math.pi/2,0.,0.,.1))
        self.assertTrue(facing.advance_allowed(0,1,math.pi/2,0.,0.,.1))
        self.assertEqual(facing.phase,'aligned')
        self.assertFalse(facing.advance_allowed(0,1,math.pi/2-.3,.5,0.,.1))

    def test_no_turn_until_measured_speed_and_held_position_settle(self):
        facing=FacingTravel();facing.reset(0.)
        for speed,error in ((.8,0.),(0.,.4),(None,0.)):
            for _ in range(30):
                self.assertFalse(facing.advance_allowed(0,1,0.,speed,error,.1))
                self.assertEqual(facing.yaw,0.)
        for _ in range(8):facing.advance_allowed(0,1,0.,0.,0.,.1)
        self.assertGreater(facing.yaw,0.)

    def test_retarget_during_turn_does_not_jump_or_reverse_angular_velocity(self):
        facing=FacingTravel();facing.reset(0.)
        for _ in range(10):facing.advance_allowed(0,1,facing.yaw,0.,0.,.1)
        before=facing.yaw
        for _ in range(120):
            facing.advance_allowed(0,-1,facing.yaw,0.,0.,.1)
            self.assertLessEqual(abs(facing.yaw-before),MAX_RATE*.1+1e-9)
            before=facing.yaw
        self.assertAlmostEqual(wrap_angle(facing.yaw+math.pi/2),0.)

    def test_stationary_or_vertical_motion_holds_heading(self):
        facing=FacingTravel();facing.reset(.7)
        for _ in range(10):self.assertTrue(facing.advance_allowed(0,0,.8,None,0.,.1))
        self.assertEqual(facing.yaw,.7)
        self.assertEqual(facing.phase,'holding')

    def test_continuous_curve_uses_exit_tolerance_but_initial_alignment_stays_strict(self):
        facing=FacingTravel();facing.reset(0.)
        for _ in range(6):facing.advance_allowed(1,0,0.,0.,0.,.1,continuous=True)
        self.assertEqual(facing.phase,'following')
        tangent=math.radians(7)
        self.assertTrue(facing.advance_allowed(math.cos(tangent),math.sin(tangent),
                                              0.,.7,.1,.1,continuous=True))
        self.assertEqual(facing.phase,'following')
        tangent=math.radians(11)
        self.assertFalse(facing.advance_allowed(math.cos(tangent),math.sin(tangent),
                                               0.,.7,.1,.1,continuous=True))
        self.assertEqual(facing.phase,'following')
        # Starting from rest still requires actual alignment within five degrees.
        facing=FacingTravel();facing.reset(0.)
        tangent=math.radians(7)
        for _ in range(40):
            self.assertFalse(facing.advance_allowed(math.cos(tangent),math.sin(tangent),
                                                   0.,0.,0.,.1,continuous=True))

    def test_changed_tangent_and_measured_yaw_cannot_bypass_commanded_heading_alignment(self):
        facing=FacingTravel();facing.reset(0.)
        for _ in range(6):facing.advance_allowed(1,0,0.,0.,0.,.1,continuous=True)
        self.assertFalse(facing.advance_allowed(0,1,math.pi/2,.5,.1,.1,continuous=True))
        self.assertEqual(facing.phase,'following')
        self.assertLessEqual(abs(facing.yaw),MAX_RATE*.1)

    def test_tracking_pause_finishes_existing_turn_without_resetting_alignment(self):
        facing=FacingTravel();facing.reset(0.)
        for _ in range(6):facing.advance_allowed(1,0,0.,0.,0.,.1,continuous=True)
        tangent=math.radians(7)
        before=facing.yaw
        self.assertTrue(facing.advance_allowed(math.cos(tangent),math.sin(tangent),
                                              0.,.5,.1,.1,continuous=True))
        previous_rate=(facing.yaw-before)/.1
        for _ in range(15):
            before=facing.yaw;facing.pause_advance(.1)
            rate=(facing.yaw-before)/.1
            self.assertLessEqual(abs(rate),MAX_RATE+1e-9)
            self.assertLessEqual(abs(rate-previous_rate)/.1,MAX_ACCEL+1e-9)
            previous_rate=rate
            self.assertEqual(facing.phase,'following')
        self.assertAlmostEqual(facing.yaw,tangent)
        self.assertAlmostEqual(facing.goal,tangent)
        # A pause does not let the next move bypass measured alignment.
        self.assertFalse(facing.advance_allowed(math.cos(tangent),math.sin(tangent),
                                               tangent+.3,0.,0.,.1,continuous=True))

    def test_step_limits_the_next_command_lead_not_only_its_start(self):
        sx,sy=limited_horizontal_step(1,1,(.25,0),(0,0),.1,1.)
        self.assertLessEqual(math.hypot(.25+sx,sy),MAX_COMMAND_LEAD+1e-9)
        self.assertLessEqual(math.hypot(sx,sy),.1)
        self.assertEqual(limited_horizontal_step(1,0,(.4,0),(0,0),.1,1.),(0.,0.))


class CoreFacingTests(unittest.TestCase):
    def prepared(self):
        core=DashCore();pos=Pos(10,0,0,0,0,vx=0.,vy=0.)
        fc=Fc(10,True,False,True,True)
        core.command(10,'pilot','claim')
        core.command(10,'pilot','engage',pos=pos,fc=fc,alt=.5)
        core.step(10,pos,fc)
        fc.armed=True;fc.landed=False;pos.z=.5
        core.step(10,pos,fc)
        return core,pos,fc

    def step(self,c,p,f,t):
        p.t=f.t=t;c.heartbeat(t,'pilot');return c.step(t,p,f)

    def test_sideways_go_turns_without_xy_then_advances_along_its_tangent(self):
        c,p,f=self.prepared();c.command(10,'pilot','goto',pos=p,fc=f,forward=0,left=1)
        original=list(c.cmd)
        moved=False
        for tick in range(1,70):
            t=10+tick*.1
            out=self.step(c,p,f,t)
            if c.cmd[:2]!=original[:2]:
                moved=True
                self.assertLessEqual(abs(wrap_angle(p.yaw-math.pi/2)),math.radians(5))
                self.assertEqual(c.cmd[0],0.)
                break
            p.yaw=c.heading.yaw
        self.assertTrue(moved)
        self.assertGreater(c.cmd[1],0.)

    def test_reversal_brakes_on_old_heading_before_turning(self):
        c,p,f=self.prepared();c.command(10,'pilot','goto',pos=p,fc=f,forward=1,left=0)
        for tick in range(1,11):
            self.step(c,p,f,10+tick*.1);p.x,p.y=c.cmd[:2];p.yaw=c.heading.yaw
        self.assertGreater(c.cmd[0],0.)
        p.vx=.8
        c.command(11,'pilot','goto',pos=p,fc=f,forward=-1,left=0)
        held=tuple(c.cmd[:2]);yaw=c.heading.yaw
        for tick in range(1,11):
            self.step(c,p,f,11+tick*.1)
            self.assertEqual(tuple(c.cmd[:2]),held);self.assertEqual(c.heading.yaw,yaw)

    @LEGACY_HOLDS
    def test_missing_velocity_blocks_horizontal_but_not_climb_and_heartbeat_still_stops(self):
        c,p,f=self.prepared();p.vx=p.vy=None
        c.command(10,'pilot','goto',pos=p,fc=f,forward=0,left=1)
        before=list(c.cmd)
        self.step(c,p,f,10.1)
        self.assertEqual(c.cmd[:2],before[:2]);self.assertGreater(c.cmd[2],before[2])
        p.t=f.t=12
        self.assertIsNone(c.step(12,p,f));self.assertEqual(c.mode,'failsafe')


if __name__=='__main__':unittest.main()

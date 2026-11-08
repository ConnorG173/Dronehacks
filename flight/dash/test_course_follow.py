"""Synthetic course execution: never connects to a drone."""
import json
import math
from pathlib import Path
import unittest
import runpy

from course_follow import CourseRun
from course_state import CourseState
from dash_core import DashCore, DashParams, Pos, Fc
from heading_control import MAX_RATE,MAX_ACCEL
import os as _os
from unittest.mock import patch as _patch
# These tests check the dashboard's in-flight holds, now off by default (DashParams.no_holds).
LEGACY_HOLDS = _patch.dict(_os.environ, {'DASH_LEGACY_HOLDS': '1'})


class CourseFollowTests(unittest.TestCase):
    def test_hardware_probe_import_is_inert(self):
        with self.assertRaises(unittest.SkipTest):
            runpy.run_path(str(Path(__file__).parent/'test_dash_live.py'),run_name='test_dash_live')

    def setup_run(self, points=None):
        c = DashCore()
        c.mode, c.owner, c.owner_hb, c.was_offboard = 'engaged', 'pilot', 1., True
        c.home = (0.,0.,0.,0.)
        c.cmd = c.target = [0.,0.,1.]
        c.last_step = 1.
        c.heading.reset(0.)
        p, f = Pos(1.,0.,0.,1.,0.,True,0.,0.), Fc(1.,True,True,True,False)
        points = points or [[0,0,1],[2,0,1],[2,1,1],[2,1,1.2]]
        path = dict(points=points, times_s=list(range(len(points))), reference=('test',1,1),
                    anchor=dict(origin_enu=[0,0,0],yaw_rad=0))
        return c,p,f,path

    def test_start_checks_reference_radius_hover_state_and_preserves_command(self):
        c,p,f,path = self.setup_run()
        before = list(c.cmd)
        self.assertFalse(c.start_course(1,path,1.5,p,f)[0])
        self.assertEqual(c.fence_radius(),1.5)
        for radius in (None,True,math.nan,math.inf,-1,21):
            self.assertFalse(c.start_course(1,path,radius,p,f)[0])
        f.armed=False
        self.assertFalse(c.start_course(1,path,6,p,f)[0])
        f.armed=True;p.vx=.3
        self.assertFalse(c.start_course(1,path,6,p,f)[0])
        p.vx=0.;p.z=.5
        self.assertFalse(c.start_course(1,path,6,p,f)[0])
        p.z=1.;p.t=-1
        self.assertFalse(c.start_course(1,path,6,p,f)[0])
        p.t=1.;c.home=(0,0,0,1)
        self.assertFalse(c.start_course(1,path,6,p,f)[0])
        c.home=(0,0,0,0)
        self.assertTrue(c.start_course(1,path,6,p,f)[0])
        self.assertEqual(c.cmd,before)
        self.assertEqual(c.fence_radius(),6)
        self.assertEqual(c.p.fence_r_m,1.5)
        self.assertFalse(c.start_course(1,path,6,p,f)[0])

    def test_full_original_route_completes_without_skipping_speed_or_yaw_limits(self):
        state=CourseState(Path(__file__).parent/'course.json')
        self.assertTrue(state.capture_anchor([0,0,0],0)[0])
        path=state.original_path()  # Offline comparison of the original controller.
        self.assertEqual(len(path['points']),1394)
        c,p,f,_=self.setup_run()
        self.assertTrue(c.start_course(1,path,6,p,f)[0])
        self.assertAlmostEqual(c.p.course_v_xy,1.0)
        self.assertGreaterEqual(c.course_run.times[-1],28.16)
        self.assertLess(c.course_run.times[-1],28.17)
        previous_yaw=0.
        for tick in range(1,6001):
            now=1+tick*.1
            p.t=f.t=now;c.heartbeat(now,'pilot')
            before=list(c.cmd)
            out=c.step(now,p,f)
            self.assertIsNotNone(out)
            self.assertLessEqual(math.dist(before[:2],c.cmd[:2]),.1200001)
            self.assertLessEqual(abs(before[2]-c.cmd[2]),.0300001)
            yaw=c.heading.yaw
            self.assertLessEqual(abs(yaw-previous_yaw),MAX_RATE*.1+1e-6)
            previous_yaw=yaw
            p.vx,p.vy=(c.cmd[0]-p.x)/.1,(c.cmd[1]-p.y)/.1
            p.x,p.y,p.z=c.cmd
            p.yaw=yaw
            if not c.course_run.active:break
        self.assertEqual(c.course_run.state,'complete',c.course_run.status())
        # Tightening initial alignment must not create repeated stops at every
        # small bend. This is an ideal follower benchmark, not flight timing.
        self.assertLess(tick*.1,100.)
        self.assertLess(math.dist(c.cmd,path['points'][-1]),1e-6)
        self.assertEqual(c.mode,'engaged')
        for tick2 in range(5):
            now+=.1;p.t=f.t=now;c.heartbeat(now,'pilot')
            self.assertEqual(c.step(now,p,f)[1],tuple(path['points'][-1]))

    @LEGACY_HOLDS
    def test_lag_pauses_clock_and_loss_never_resumes(self):
        c,p,f,path=self.setup_run()
        self.assertTrue(c.start_course(1,path,6,p,f)[0])
        run=c.course_run
        for _ in range(10):
            run.target(.1,[0,0,1],[.3,0,1])
        self.assertEqual(run.elapsed,0)
        self.assertIsNone(run.pending)
        p.t=f.t=2.1
        self.assertIsNone(c.step(2.1,p,f))
        self.assertFalse(run.active)
        c.heartbeat(2.2,'pilot');p.t=f.t=2.2
        self.assertIsNone(c.step(2.2,p,f))
        self.assertEqual(c.mode,'failsafe')

    def test_curve_keeps_moving_while_turning_with_delayed_measured_heading(self):
        angles=[i*math.pi/240 for i in range(121)]
        points=[[math.sin(a),1-math.cos(a),1.] for a in angles]
        c,p,f,path=self.setup_run(points)
        path['times_s']=[a/math.radians(40) for a in angles]
        self.assertTrue(c.start_course(1,path,6,p,f)[0])
        moving_turns=0
        for tick in range(1,601):
            now=1+tick*.1;p.t=f.t=now;c.heartbeat(now,'pilot')
            before=list(c.cmd);yaw_before=c.heading.yaw
            self.assertIsNotNone(c.step(now,p,f))
            step=math.dist(before[:2],c.cmd[:2])
            progress=c.course_run.elapsed/c.course_run.times[-1]
            if .05<progress<.95:
                self.assertGreater(step,1e-7,'smooth curve unexpectedly stopped')
                self.assertEqual(c.heading.phase,'following')
            if step>1e-7:
                tangent=math.atan2(c.cmd[1]-before[1],c.cmd[0]-before[0])
                error=math.atan2(math.sin(tangent-p.yaw),math.cos(tangent-p.yaw))
                self.assertLessEqual(abs(error),math.radians(10)+1e-8)
                if abs(c.heading.yaw-yaw_before)>1e-5:moving_turns+=1
            # Pure synthetic position and heading lag, not a vehicle model.
            following=[a+(1-math.exp(-.1/.15))*(b-a)
                       for a,b in zip((p.x,p.y,p.z),c.cmd)]
            p.vx,p.vy=(following[0]-p.x)/.1,(following[1]-p.y)/.1
            p.x,p.y,p.z=following
            yaw_error=math.atan2(math.sin(c.heading.yaw-p.yaw),math.cos(c.heading.yaw-p.yaw))
            p.yaw+=(1-math.exp(-2.8*.1))*yaw_error
            if not c.course_run.active:break
        self.assertEqual(c.course_run.state,'complete')
        self.assertGreater(moving_turns,20)

    def test_course_speed_is_separate_from_manual_speed_and_rejects_bad_values(self):
        for value in (math.nan,math.inf,-math.inf,0,-.1,True):
            with self.assertRaises(ValueError):DashParams(course_v_xy=value)
        c,p,f,path=self.setup_run()
        self.assertEqual(c.p.v_xy,1.)
        self.assertTrue(c.start_course(1,path,6,p,f)[0])
        self.assertEqual(c.course_run.xy_speed,1.0)
        self.assertEqual(c.status(1)['params']['course_v_xy'],1.0)

    def test_slow_curve_completion_finishes_yaw_before_entering_hold(self):
        angles=[i*math.pi/120 for i in range(61)]
        c,p,f,path=self.setup_run([[.05*math.sin(a),.05*(1-math.cos(a)),1.] for a in angles])
        path['times_s']=[a/(math.pi/4) for a in angles]
        self.assertTrue(c.start_course(1,path,6,p,f)[0])
        previous_rate=0.;completed_tick=None;tracker=None
        for tick in range(1,201):
            now=1+tick*.1;p.t=f.t=now;c.heartbeat(now,'pilot')
            before_yaw=c.heading.yaw
            tracker=c.heading.course_turn or tracker
            c.step(now,p,f)
            rate=(c.heading.yaw-before_yaw)/.1
            self.assertLessEqual(abs(rate-previous_rate)/.1,MAX_ACCEL+1e-7)
            previous_rate=rate
            p.vx,p.vy=(c.cmd[0]-p.x)/.1,(c.cmd[1]-p.y)/.1
            p.x,p.y,p.z=c.cmd;p.yaw=c.heading.yaw
            if not c.course_run.active:
                self.assertIsNotNone(tracker)
                self.assertAlmostEqual(tracker.rate,0.)
                if completed_tick is None:completed_tick=tick
                if tick-completed_tick>=5:break
        self.assertEqual(c.course_run.state,'complete')
        self.assertIsNotNone(completed_tick)

    def test_tracking_pause_resumes_without_another_turn_but_checks_measured_yaw(self):
        for yaw_error in (0.,math.radians(15)):
            c,p,f,path=self.setup_run(points=[[0,0,1],[2,0,1]])
            self.assertTrue(c.start_course(1,path,6,p,f)[0])
            # A stationary synthetic drone makes the 20 cm pacing guard pause.
            for tick in range(1,21):
                now=1+tick*.1;p.t=f.t=now;c.heartbeat(now,'pilot');c.step(now,p,f)
            self.assertEqual(c.heading.phase,'following')
            self.assertIsNone(c.course_run.pending)
            self.assertGreater(c.cmd[0],.2)
            held=list(c.cmd);elapsed=c.course_run.elapsed
            now+=.1;p.t=f.t=now;c.heartbeat(now,'pilot');c.step(now,p,f)
            self.assertEqual(c.cmd,held)
            self.assertEqual(c.course_run.elapsed,elapsed)
            # Catch up to the held point; only aligned measured yaw may resume.
            p.x,p.y,p.z=c.cmd;p.yaw=yaw_error
            now+=.1;p.t=f.t=now;c.heartbeat(now,'pilot');c.step(now,p,f)
            if yaw_error:
                self.assertEqual(c.cmd,held)
                self.assertEqual(c.course_run.elapsed,elapsed)
            else:
                self.assertGreater(c.cmd[0],held[0])
                self.assertGreater(c.course_run.elapsed,elapsed)

    def test_nonrepresentable_timing_is_rejected_before_start(self):
        c,p,f,path=self.setup_run()
        path['times_s']=[-1e308,-1.,1.,1e308]
        self.assertFalse(c.start_course(1,path,6,p,f)[0])
        self.assertIsNone(c.course_run)

    def test_manual_override_kill_and_reset_do_not_resume_or_jump(self):
        for op,kw in [('hold',{}),('land',{}),('release',{}),('disarm',{}),
                      ('nudge',dict(fwd=.25)),('goto',dict(forward=1,left=0)),('alt',dict(alt=1.1))]:
            c,p,f,path=self.setup_run()
            self.assertTrue(c.start_course(1,path,6,p,f)[0])
            self.assertFalse(c.command(1,'viewer',op,p,f,**kw)[0])
            self.assertTrue(c.course_run.active)
            before=list(c.cmd)
            self.assertTrue(c.command(1,'pilot',op,p,f,**kw)[0])
            self.assertFalse(c.course_run.active)
            self.assertEqual(c.cmd,before)
            self.assertEqual(c.fence_radius(),6)
            if op=='goto':self.assertEqual(c.target,[1,0,1])
        f.armed=False;f.landed=True;p.z=0
        self.assertTrue(c.command(1,'pilot','reset',p,f)[0])
        self.assertEqual(c.fence_radius(),1.5)

    @LEGACY_HOLDS
    def test_position_jump_and_offboard_exit_cancel(self):
        for change in ('jump','offboard'):
            c,p,f,path=self.setup_run()
            self.assertTrue(c.start_course(1,path,6,p,f)[0])
            if change=='jump':p.x=1
            else:f.offboard_active=False
            self.assertIsNone(c.step(1.1,p,f))
            self.assertFalse(c.course_run.active)

    def test_offline_original_transform_does_not_use_display_geometry(self):
        state=CourseState(Path(__file__).parent/'course.json')
        state.capture_anchor([4,-3,-.8],math.pi/2)
        state._adjusted_world=[[99,99,99],[100,100,100]]
        path=state.original_path()
        self.assertAlmostEqual(path['points'][0][2],.2)
        self.assertAlmostEqual(path['points'][-1][0],5.7069)
        self.assertAlmostEqual(path['points'][-1][1],-2.5276)
        self.assertEqual(len(path['points']),1394)
        reference=path['reference']
        state.configure_preview()
        self.assertEqual(state.execution_reference(),reference)
        state.invalidate_anchor('test interruption')
        self.assertIsNone(state.execution_reference())


if __name__=='__main__':unittest.main()

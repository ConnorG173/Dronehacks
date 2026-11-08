"""Offline full-course and obstacle checks with an ideal kinematic follower.

No ROS imports, network, arming commands or aircraft access. This is not a
physics simulation or flight qualification.
"""
import json
import math
from pathlib import Path
import tempfile

from course_state import CourseState
from course_safety import observations, segment_clearance
from dash_core import DashCore, Pos, Fc


def run(controller, obstacle=False):
    with tempfile.TemporaryDirectory() as temporary:
        state=CourseState(Path(temporary)/'course.json' if obstacle else Path(__file__).with_name('course.json'))
        if obstacle:state.import_route([[0,0,1],[4,0,1]])
        state.capture_anchor([0,0,0],0)
        depth=dict(status='live',age_s=.1,cell_m=.08,points=[[2.,0.]] if obstacle else [[20.,20.]])
        state.update_preview(depth,now=10)
        path=state.execution_path(10)
        core=DashCore();core.owner='offline';core.owner_hb=10;core.home=(0,0,0,0)
        core.course_controller=controller
        pos=Pos(10,0,0,0,0,True,0,0,0);fc=Fc(10,True,False,True,True)
        ok,reason=core.prepare_course(10,path,6,pos,fc,depth)
        if not ok:raise AssertionError(reason)
        # Synthetic state change only. No VehicleCommand or publisher exists.
        fc.armed=True
        walls,cell,_=observations(depth)
        minimum=math.inf;maximum_speed=0.;peak_offset=0.;phase_seen=set()
        for tick in range(1,6001):
            now=10+tick*.1;pos.t=fc.t=now;core.heartbeat(now,'offline')
            previous=(pos.x,pos.y,pos.z)
            output=core.step(now,pos,fc,depth)
            if output is None:raise AssertionError(core.reason)
            phase_seen.add(core.launch_state)
            kind,values=output
            if kind=='pose':
                pos.x,pos.y,pos.z=values;pos.yaw=core.heading.yaw
            else:
                pos.x+=values[0]*.1;pos.y+=values[1]*.1;pos.z+=values[2]*.1
                pos.yaw+=core.course_yaw_rate*.1
            pos.vx,pos.vy,pos.vz=((b-a)/.1 for a,b in zip(previous,(pos.x,pos.y,pos.z)))
            if pos.z>.32:fc.landed=False
            minimum=min(minimum,segment_clearance(previous,(pos.x,pos.y),walls,cell))
            maximum_speed=max(maximum_speed,math.hypot(pos.vx,pos.vy))
            peak_offset=max(peak_offset,abs(pos.y))
            if core.course_run is not None and not core.course_run.active:break
        assert core.course_run is not None and core.course_run.state=='complete', core.status(now)
        assert {'taking_off','settling','flying'} <= phase_seen,phase_seen
        assert minimum>=.3-1e-9,minimum
        assert maximum_speed<=.35+1e-6,maximum_speed
        if obstacle:assert peak_offset>.4,peak_offset
        return dict(controller=controller,scenario='wall_detour' if obstacle else 'saved_full_course',
            completed=True,seconds=round(tick*.1,2),original_points=state.snapshot(10)['original_count'],
            flight_points=len(path['points']),source=path['source'],min_wall_clearance_m=round(minimum,4),
            max_xy_speed_mps=round(maximum_speed,4),launch_states=sorted(phase_seen))


if __name__=='__main__':
    print(json.dumps(dict(model='ideal kinematics; not hardware qualification',
        scenarios=[run(controller,obstacle) for controller in ('smooth_position','velocity_tracker')
                   for obstacle in (False,True)]),indent=2))

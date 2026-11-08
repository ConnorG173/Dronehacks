"""Turn-before-advance heading policy for dashboard position ramps.

No ROS or flight commands. Quintic turns use the shortest angle, with bounded
commanded yaw rate/acceleration and zero endpoint rate/acceleration. Position
advance is separately gated on measured heading and settling telemetry.
"""
from dataclasses import dataclass
import math

from path_heading import wrap_angle
from yaw_tracker import YawTracker


MAX_RATE = math.radians(90)
MAX_ACCEL = math.radians(180)
ENTER_ERROR = math.radians(5)
EXIT_ERROR = math.radians(10)
SETTLE_SPEED = .15
SETTLE_POSITION = .15
SETTLE_SECONDS = .2
MAX_COMMAND_LEAD = .30


@dataclass
class YawTurn:
    start: float
    target: float
    elapsed: float = 0.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.start,self.target)):
            raise ValueError('yaw turn requires finite angles')
        self.delta = wrap_angle(self.target-self.start)
        self.duration = (max(.2,1.875*abs(self.delta)/MAX_RATE,
                             math.sqrt((10/math.sqrt(3))*abs(self.delta)/MAX_ACCEL))
                         if abs(self.delta)>1e-10 else 0.)
        self.target = self.start+self.delta
        self.yaw, self.rate, self.accel = self.start, 0., 0.

    @property
    def done(self):
        return self.elapsed >= self.duration

    def step(self, dt):
        self.elapsed = min(self.duration,self.elapsed+max(0.,min(float(dt),.2)))
        if self.done:
            self.yaw,self.rate,self.accel=self.target,0.,0.
            return self.yaw
        u=self.elapsed/self.duration
        self.yaw=self.start+self.delta*(10*u**3-15*u**4+6*u**5)
        self.rate=self.delta*(30*u**2-60*u**3+30*u**4)/self.duration
        self.accel=self.delta*(60*u-180*u**2+120*u**3)/self.duration**2
        return self.yaw


class FacingTravel:
    def __init__(self):
        self.yaw = self.goal = self.error = None
        self.turn = None
        self.course_turn = None
        self.pace = 1.
        self.phase = 'holding'
        self.reason = 'Holding heading'
        self.settled = 0.
        self.fixed = False   # True: keep the current heading; translate without turning

    def reset(self, yaw):
        if not math.isfinite(yaw):
            raise ValueError('measured heading must be finite')
        self.yaw=float(yaw)
        self.goal=self.error=self.turn=None
        self.course_turn=None
        self.pace=1.
        self.phase,self.reason,self.settled='holding','Holding heading',0.

    def hold(self):
        # Explicit Hold/landing cancels further turning at the last command.
        self.goal=self.error=self.turn=None
        self.course_turn=None
        self.pace=1.
        self.phase,self.reason,self.settled='holding','Holding heading',0.

    def pause_advance(self, dt):
        """Pause course translation without discarding its heading alignment.

        Finish only the bounded turn already issued for the previous tangent.
        A new translation still goes through advance_allowed and measured yaw.
        Explicit Hold and vertical/stationary route segments use hold instead.
        """
        if self.course_turn is not None:
            self.yaw=self.course_turn.step(self.goal,dt)
        elif self.turn is not None and not self.turn.done:
            self.yaw=self.turn.step(dt)
        self.reason='Waiting for the drone to catch up'

    def advance_allowed(self, dx, dy, measured_yaw, speed, position_error, dt, continuous=False):
        dt=max(0.,min(float(dt),.2))
        if self.yaw is None:
            self.reset(measured_yaw)
        if self.fixed:
            # Hold-start-heading mode: no settle/turn/align, just translate.
            self.goal=self.error=self.turn=self.course_turn=None
            self.pace=1.
            self.phase='fixed'
            self.reason=('Waiting for the drone to catch up' if position_error>MAX_COMMAND_LEAD
                         else 'Holding the start heading')
            return position_error<=MAX_COMMAND_LEAD
        if math.hypot(dx,dy)<=1e-6:
            if continuous and self.course_turn is not None:
                # A stationary/vertical route segment keeps its last tangent.
                # Finish any remaining rotation smoothly without resetting rate.
                self.yaw=self.course_turn.step(self.goal,dt)
                self.pace=1.
                self.reason='Holding the last course tangent'
                return True
            self.hold()
            return True
        goal=math.atan2(dy,dx)
        valid_speed=speed is not None and math.isfinite(speed) and speed>=0
        if continuous and self.course_turn is not None:
            self.goal=goal
            self.error=wrap_angle(goal-measured_yaw)
            self.yaw=self.course_turn.step(goal,dt)
            error=max(abs(self.error),abs(wrap_angle(goal-self.yaw)))
            # Ease the path clock before reaching the existing heading limit.
            # The yaw tracker keeps its angular velocity across each tangent;
            # curves no longer restart the settle/turn/advance sequence.
            u=max(0.,min(1.,(error-ENTER_ERROR)/(EXIT_ERROR-ENTER_ERROR)))
            self.pace=1.-u*u*(3.-2.*u)
            self.phase='following'
            if not valid_speed:
                self.pace=0.
                self.reason='Waiting for horizontal velocity'
                return False
            if error>=EXIT_ERROR:
                self.reason='Aligning with the course tangent'
                return False
            self.reason=('Waiting for the drone to catch up' if position_error>MAX_COMMAND_LEAD else
                         'Slowing through the bend' if self.pace<.99 else
                         'Following the course tangent')
            return position_error<=MAX_COMMAND_LEAD
        if self.goal is None or abs(wrap_angle(goal-self.goal))>1e-4:
            self.goal=goal
            # Once aligned, small course-tangent changes use the normal exit
            # threshold below. Reapplying the stricter entry threshold here
            # causes repeated stop/turn cycles on an otherwise aligned curve.
            if not (continuous and self.phase=='aligned'
                    and abs(wrap_angle(goal-self.yaw))<=EXIT_ERROR):
                self.phase,self.reason,self.settled='settling','Settling before turning toward travel',0.
        self.error=wrap_angle(goal-measured_yaw)
        stopped=valid_speed and speed<=SETTLE_SPEED and position_error<=SETTLE_POSITION

        if self.phase=='aligned':
            if valid_speed and abs(self.error)<=EXIT_ERROR:
                if continuous:
                    if self.turn is None or self.turn.done:
                        self.turn=YawTurn(self.yaw,self.goal)
                    self.yaw=self.turn.step(dt)
                self.reason=('Facing travel' if position_error<=MAX_COMMAND_LEAD else
                             'Waiting for the drone to catch up')
                return position_error<=MAX_COMMAND_LEAD
            self.phase,self.reason,self.settled='settling','Settling to regain the forward view',0.

        # Finish an already-issued turn smoothly before planning a changed goal.
        # XY remains frozen while this runs; repeated target edits never jump yaw.
        if self.turn is not None and not self.turn.done:
            self.yaw=self.turn.step(dt)
            self.reason='Turning to face travel'
            return False

        if not valid_speed:
            self.phase,self.reason,self.settled='waiting','Waiting for horizontal velocity',0.
            return False
        if self.phase=='waiting':
            self.phase='settling'
        if self.phase=='settling':
            self.reason='Settling before turning toward travel'
            self.settled=self.settled+dt if stopped else 0.
            if self.settled+1e-9>=SETTLE_SECONDS:
                self.turn=YawTurn(self.yaw,self.goal)
                self.phase,self.reason,self.settled='turning','Turning to face travel',0.
            return False

        self.phase='turning'
        # Commanded turn completion alone is insufficient: wait for the actual
        # heading, low measured speed and nearby held position to agree.
        aligned=stopped and abs(self.error)<=ENTER_ERROR
        self.settled=self.settled+dt if aligned else 0.
        self.reason='Waiting for measured heading to align'
        if self.settled+1e-9>=SETTLE_SECONDS:
            self.phase,self.reason='aligned','Facing travel'
            if continuous:
                self.course_turn=YawTracker(self.yaw,max_rate=MAX_RATE,max_accel=MAX_ACCEL)
            return True
        return False

    def status(self):
        degrees=lambda angle: None if angle is None else round(math.degrees(wrap_angle(angle)),1)
        return dict(policy='hold_start' if self.fixed else 'path_tangent',phase=self.phase,reason=self.reason,
                    commanded_deg=degrees(self.yaw),target_deg=degrees(self.goal),
                    error_deg=degrees(self.error),max_rate_dps=math.degrees(MAX_RATE),
                    max_accel_dps2=math.degrees(MAX_ACCEL),course_tracking='continuous',
                    course_pace=round(self.pace,3))

    @property
    def course_heading_settled(self):
        return (self.course_turn is None or
                (abs(self.course_turn.rate)<=1e-6 and
                 abs(wrap_angle(self.goal-self.yaw))<=1e-6))


def limited_horizontal_step(dx,dy,cmd_xy,position_xy,dt,speed_limit):
    """Advance along the requested tangent without leading the drone by >30cm."""
    distance=math.hypot(dx,dy)
    if distance<=1e-12:return 0.,0.
    ux,uy=dx/distance,dy/distance
    ex,ey=cmd_xy[0]-position_xy[0],cmd_xy[1]-position_xy[1]
    if math.hypot(ex,ey)>MAX_COMMAND_LEAD+1e-9:return 0.,0.
    along=ex*ux+ey*uy
    perpendicular2=max(0.,ex*ex+ey*ey-along*along)
    available=max(0.,-along+math.sqrt(max(0.,MAX_COMMAND_LEAD**2-perpendicular2)))
    step=min(distance,max(0.,speed_limit*dt),available)
    return ux*step,uy*step

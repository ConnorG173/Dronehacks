"""PX4 ROS messages -> MAVLink, preserving the bridge's existing FRD values."""
import math

from pymavlink.dialects.v20 import common as mav

DIAGONAL = (0, 6, 11, 15, 18, 20)
MAX_SAMPLE_AGE_US = 250_000
MAX_PUBLISH_AGE_US = 250_000


def odometry_metadata(msg):
    # ROS fixed-size arrays contain numpy.float32, which JSON cannot encode.
    return dict(sample_us=int(msg.timestamp_sample), publish_us=int(msg.timestamp),
                position=[float(v) for v in msg.position], q=[float(v) for v in msg.q],
                reset_counter=int(msg.reset_counter))


def fresh(stamp_us, now_us, max_age_us=MAX_PUBLISH_AGE_US):
    return int(stamp_us) > 0 and -50_000 <= now_us - int(stamp_us) <= max_age_us


def covariance(linear, angular):
    values = [float(v) for v in list(linear) + list(angular)]
    if len(values) != 6:
        raise ValueError('Expected six covariance diagonal values')
    result = [0.0] * 21
    for index, value in zip(DIAGONAL, values):
        if math.isinf(value) or (math.isfinite(value) and value < -1e-12):
            raise ValueError('Invalid covariance diagonal')
        result[index] = max(0.0, value) if math.isfinite(value) else float('nan')
    return result


def odometry_message(msg, now_us):
    if not fresh(msg.timestamp, now_us) or not fresh(msg.timestamp_sample, now_us, MAX_SAMPLE_AGE_US):
        raise ValueError('Stale/future vision sample')
    if msg.pose_frame != 2 or msg.velocity_frame != 3:
        raise ValueError('Expected local FRD pose and body FRD velocity')
    if not all(math.isfinite(float(v)) for v in list(msg.position) + list(msg.q) + list(msg.velocity)):
        raise ValueError('Invalid vision position, quaternion or velocity')
    if abs(sum(float(v)**2 for v in msg.q) - 1) > .002:
        raise ValueError('Vision quaternion is not normalized')
    # No second ENU/NED rotation: px4_odom_bridge already did the conversion.
    return mav.MAVLink_odometry_message(
        int(msg.timestamp_sample), mav.MAV_FRAME_LOCAL_FRD, mav.MAV_FRAME_BODY_FRD,
        *[float(v) for v in msg.position], [float(v) for v in msg.q],
        *[float(v) for v in msg.velocity], *[float(v) for v in msg.angular_velocity],
        covariance(msg.position_variance, msg.orientation_variance),
        covariance(msg.velocity_variance, [float('nan')] * 3),
        int(msg.reset_counter) & 255, mav.MAV_ESTIMATOR_TYPE_VIO, int(msg.quality))


def setpoint_message(msg, mode, now_us, target_system=12):
    if int(msg.timestamp) != int(mode.timestamp):
        raise ValueError('Setpoint and control-mode timestamps do not match')
    if not fresh(msg.timestamp, now_us):
        raise ValueError('Stale/future setpoint')
    if any(bool(getattr(mode, name, False)) for name in
           ('attitude', 'body_rate', 'thrust_and_torque', 'direct_actuator')):
        raise ValueError('Unsupported control mode')
    mask, values, active = 0, [], []
    for group, shift in [('position', 0), ('velocity', 3), ('acceleration', 6)]:
        group_values = [float(v) for v in getattr(msg, group)]
        finite = [math.isfinite(v) for v in group_values]
        if len(group_values) != 3 or (any(finite) and not all(finite)):
            raise ValueError('Partial/non-finite vector: ' + group)
        if any(math.isinf(v) for v in group_values):
            raise ValueError('Infinite setpoint')
        enabled = bool(getattr(mode, group))
        if enabled != all(finite):
            raise ValueError('Control flags disagree with setpoint: ' + group)
        if enabled:
            active.append(group)
            values.extend(group_values)
        else:
            mask |= 7 << shift
            values.extend([0.0] * 3)
    if not active:
        raise ValueError('Empty control setpoint')
    for value, bit in [(float(msg.yaw), 10), (float(msg.yawspeed), 11)]:
        if math.isinf(value):
            raise ValueError('Infinite yaw command')
        if math.isnan(value):
            mask |= 1 << bit
            value = 0.0
        values.append(value)
    # PX4 v1.16.2 timestamps receipt locally for this message; time_boot_ms is
    # not used to timestamp the incoming TrajectorySetpoint.
    return mav.MAVLink_set_position_target_local_ned_message(
        (now_us // 1000) & 0xffffffff, target_system, 1, mav.MAV_FRAME_LOCAL_NED,
        mask, *values)


class SetpointPair:
    """Allow either DDS callback order; never repeat a consumed control tick."""
    def __init__(self):
        self.mode = self.setpoint = None
        self.last_stamp = None

    def update(self, kind, msg, now_us):
        setattr(self, kind, msg)
        if self.mode is None or self.setpoint is None:
            return None
        if self.mode.timestamp != self.setpoint.timestamp:
            return None
        if self.last_stamp is not None and self.setpoint.timestamp <= self.last_stamp:
            return None
        result = setpoint_message(self.setpoint, self.mode, now_us)
        self.last_stamp = self.setpoint.timestamp
        return result

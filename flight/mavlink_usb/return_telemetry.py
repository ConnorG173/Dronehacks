"""PX4 MAVLink -> private ROS containers, and explicit operator commands.

Only the fields assigned below are provided. These are not replicas of the
uORB topics: reset counters, fusion flags, estimator validity and preflight
checks are not available here and must not be inferred from default fields.
Position validity means finite NED pose data, not an EKF-health assessment.
``timestamp`` is host receipt time; sample timestamps retain the FC clock.

Mode mapping follows PX4 v1.16.2 commander/px4_custom_mode.h; heartbeat
failsafe mapping follows its mavlink/streams/HEARTBEAT.hpp.
"""
import math

from pymavlink.dialects.v20 import common as mav
from px4_msgs.msg import (BatteryStatus, SensorCombined, VehicleAttitude,
                         VehicleCommandAck, VehicleLandDetected,
                         VehicleLocalPosition, VehicleStatus)


TOPIC_TYPES = {
    'vehicle_local_position': VehicleLocalPosition,
    'vehicle_status_v1': VehicleStatus,
    'vehicle_land_detected': VehicleLandDetected,
    'battery_status': BatteryStatus,
    'vehicle_attitude': VehicleAttitude,
    'sensor_combined': SensorCombined,
    'vehicle_command_ack': VehicleCommandAck,
}


def quaternion(values):
    q = [float(v) for v in values]
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('nonfinite quaternion')
    norm = math.sqrt(sum(v*v for v in q))
    if not .9 <= norm <= 1.1:
        raise ValueError('invalid quaternion norm')
    return [v/norm for v in q]


def nav_state(custom_mode):
    main = (int(custom_mode) >> 16) & 255
    sub = (int(custom_mode) >> 24) & 255
    if main == 3:
        return {0: 2, 1: 21, 2: 6}.get(sub, 31)
    if main == 4:
        # AUTO_LAND also represents DESCEND on the wire; both are non-Offboard.
        return {2: 17, 3: 4, 4: 3, 5: 5, 6: 18, 8: 19,
                9: 20, 10: 22, **{11+i: 23+i for i in range(8)}}.get(sub, 31)
    return {1: 0, 2: 1, 5: 10, 6: 14, 7: 15, 10: 13}.get(main, 31)


def convert(message, receipt_wall_us):
    """Return (private topic suffix, ROS message), None, or raise ValueError."""
    if (message.get_srcSystem(), message.get_srcComponent()) != (12, 1):
        return None
    kind = message.get_type()
    if kind == 'ODOMETRY':
        if (message.frame_id, message.child_frame_id, message.estimator_type) != (
                mav.MAV_FRAME_LOCAL_NED, mav.MAV_FRAME_LOCAL_NED,
                mav.MAV_ESTIMATOR_TYPE_AUTOPILOT):
            raise ValueError('expected PX4 NED autopilot odometry')
        q = quaternion(message.q)
        if not all(math.isfinite(v) for v in (message.x, message.y, message.z)):
            raise ValueError('nonfinite local position')
        out = VehicleLocalPosition()
        out.timestamp_sample = int(message.time_usec)
        out.x, out.y, out.z = float(message.x), float(message.y), float(message.z)
        out.vx, out.vy, out.vz = float(message.vx), float(message.vy), float(message.vz)
        out.xy_valid = out.z_valid = True
        out.v_xy_valid = all(math.isfinite(v) for v in (message.vx, message.vy))
        out.v_z_valid = math.isfinite(message.vz)
        w, x, y, z = q
        out.heading = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        # No range estimate or individual reset counters exist in ODOMETRY.
        out.dist_bottom = float('nan')
        topic = 'vehicle_local_position'
    elif kind == 'HEARTBEAT':
        if message.autopilot != mav.MAV_AUTOPILOT_PX4:
            raise ValueError('expected PX4 heartbeat')
        out = VehicleStatus()
        out.arming_state = 2 if message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED else 1
        out.nav_state = (nav_state(message.custom_mode)
                         if message.base_mode & mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED else 31)
        out.failsafe = message.system_status in (
            mav.MAV_STATE_CRITICAL, mav.MAV_STATE_EMERGENCY,
            mav.MAV_STATE_FLIGHT_TERMINATION)
        topic = 'vehicle_status_v1'
    elif kind == 'EXTENDED_SYS_STATE':
        if message.landed_state not in (mav.MAV_LANDED_STATE_ON_GROUND,
                mav.MAV_LANDED_STATE_IN_AIR, mav.MAV_LANDED_STATE_TAKEOFF,
                mav.MAV_LANDED_STATE_LANDING):
            return None
        out = VehicleLandDetected()
        out.landed = message.landed_state == mav.MAV_LANDED_STATE_ON_GROUND
        topic = 'vehicle_land_detected'
    elif kind == 'SYS_STATUS':
        out = BatteryStatus()
        valid_voltage = 0 < message.voltage_battery < 65535
        out.voltage_v = message.voltage_battery / 1000.0 if valid_voltage else 0.0
        out.connected = valid_voltage
        out.current_a = message.current_battery / 100.0 if message.current_battery != -1 else -1.0
        out.remaining = message.battery_remaining / 100.0 if 0 <= message.battery_remaining <= 100 else -1.0
        out.id = 1
        topic = 'battery_status'
    elif kind == 'ATTITUDE_QUATERNION':
        out = VehicleAttitude()
        out.q = quaternion([message.q1, message.q2, message.q3, message.q4])
        out.timestamp_sample = int(message.time_boot_ms) * 1000
        topic = 'vehicle_attitude'
    elif kind == 'HIGHRES_IMU':
        if message.fields_updated & 0x38 != 0x38:
            return None  # Do not refresh gyro freshness from accelerometer-only data.
        gyro = [float(message.xgyro), float(message.ygyro), float(message.zgyro)]
        if not all(math.isfinite(v) for v in gyro):
            raise ValueError('nonfinite gyro')
        out = SensorCombined()
        out.gyro_rad = gyro
        accel = [float(message.xacc), float(message.yacc), float(message.zacc)]
        valid_accel = message.fields_updated & 7 == 7 and all(math.isfinite(v) for v in accel)
        out.accelerometer_m_s2 = accel if valid_accel else [float('nan')]*3
        out.accelerometer_timestamp_relative = 0 if valid_accel else 2147483647
        topic = 'sensor_combined'
    elif kind == 'COMMAND_ACK':
        out = VehicleCommandAck()
        out.command, out.result = int(message.command), int(message.result)
        out.result_param1, out.result_param2 = int(message.progress), int(message.result_param2)
        out.target_system, out.target_component = int(message.target_system), int(message.target_component)
        out.from_external = False  # FC generated this acknowledgment.
        topic = 'vehicle_command_ack'
    else:
        return None
    out.timestamp = int(receipt_wall_us)
    return topic, out


def command_message(message, now_us):
    """Translate one explicit, fresh allowed request; never create or retry one."""
    age = int(now_us) - int(message.timestamp)
    if age > 250_000 or age < -50_000:
        raise ValueError('stale or future operator command')
    if not message.from_external or (message.target_system, message.target_component) != (12, 1):
        raise ValueError('unexpected command destination or origin')
    if message.source_system != 12 or message.source_component not in (1, 197, 198):
        raise ValueError('unexpected command source')
    params = [float(getattr(message, 'param'+str(i))) for i in range(1, 8)]
    # PX4 Land mode (DO_SET_MODE custom AUTO=4 / LAND=6): the dashboard course + return hop lands with it
    px4_land = message.command == mav.MAV_CMD_DO_SET_MODE and tuple(params[:3]) == (1., 4., 6.)
    if not all(math.isfinite(v) for v in params) or any(v != 0 for v in params[(3 if px4_land else 2):]):
        raise ValueError('unexpected operator command parameters')
    if message.command == mav.MAV_CMD_COMPONENT_ARM_DISARM:
        if message.source_component != 197 or tuple(params[:2]) not in ((1., 0.), (0., 0.), (0., 21196.)):
            raise ValueError('only normal arm or explicit disarm requests are allowed')
    elif message.command == mav.MAV_CMD_DO_SET_MODE:
        if tuple(params[:2]) != (1., 6.) and not px4_land:
            raise ValueError('only the Offboard or PX4 Land mode request is allowed')
    else:
        raise ValueError('unsupported operator command')
    return mav.MAVLink_command_long_message(12, 1, int(message.command), 0, *params)

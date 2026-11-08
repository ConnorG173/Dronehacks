"""Request Offboard once at startup, only with fresh disarmed/landed telemetry."""
import time


def eligible(now, samples):
    keys = ('status', 'land', 'flags', 'control')
    # PX4 health/status updates can be 2 Hz; the control heartbeat is 10 Hz.
    if any(k not in samples or now - samples[k][0] > (0.5 if k == 'control' else 1.0)
           for k in keys):
        return False
    status, land, flags, control = (samples[k][1] for k in keys)
    return (status.arming_state == 1 and land.landed
            and not flags.local_position_invalid
            and not flags.local_altitude_invalid
            and not flags.offboard_control_signal_lost and control.position)


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import (VehicleStatus, VehicleLandDetected, FailsafeFlags,
                             OffboardControlMode, VehicleCommand)
    from skybeta_flight_interface.topics import fmu_topic

    rclpy.init()
    node = rclpy.create_node('dashboard_startup_mode')
    samples = {}
    for direction, base, cls, key in (
        ('out', 'vehicle_status', VehicleStatus, 'status'),
        ('out', 'vehicle_land_detected', VehicleLandDetected, 'land'),
        ('out', 'failsafe_flags', FailsafeFlags, 'flags'),
        ('in', 'offboard_control_mode', OffboardControlMode, 'control'),
    ):
        node.create_subscription(
            cls, fmu_topic(direction, base, cls.__name__),
            lambda msg, k=key: samples.update({k: (time.monotonic(), msg)}),
            qos_profile_sensor_data)
    pub = node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
    deadline, stable_since, sent_at = time.monotonic() + 90, None, None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            status = samples.get('status')
            land = samples.get('land')
            if ((status and status[1].arming_state != 1)
                    or (land and not land[1].landed)):
                node.get_logger().warn('No startup mode change: vehicle is armed or airborne')
                return 1
            if status and now - status[0] < 0.5 and status[1].nav_state == 14:
                node.get_logger().info('PX4 confirms Offboard; no arm command sent')
                return 0
            if sent_at is not None:
                if now - sent_at > 5:
                    break
                continue
            if not eligible(now, samples):
                stable_since = None
                continue
            if stable_since is None:
                stable_since = now
            if now - stable_since >= 2 and pub.get_subscription_count() > 0:
                msg = VehicleCommand()
                msg.timestamp = node.get_clock().now().nanoseconds // 1000
                msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                msg.param1, msg.param2 = 1.0, 6.0
                msg.target_system = msg.source_system = 12
                msg.target_component = 1
                msg.source_component = 198
                msg.from_external = True
                pub.publish(msg)
                sent_at = now
                node.get_logger().info('Requested Offboard once while disarmed and landed')
        node.get_logger().error('Offboard was not confirmed; no retry or arm command sent')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

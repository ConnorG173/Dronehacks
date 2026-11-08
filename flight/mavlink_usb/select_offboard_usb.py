#!/usr/bin/env python3
"""One startup mode request using USB telemetry; never arms or retries."""
import math
import time


MAX_AGE = {'status': 1.5, 'land': 1.0, 'position': .5, 'control': .5}


def fresh(now, sample, max_age):
    return sample is not None and 0 <= now - sample[0] <= max_age


def eligible(now, samples):
    if any(not fresh(now, samples.get(key), age) for key, age in MAX_AGE.items()):
        return False
    status, land, position, control = (samples[k][1] for k in MAX_AGE)
    return (status.arming_state == 1 and not status.failsafe and land.landed
            and position.xy_valid and position.z_valid
            and all(math.isfinite(v) for v in (position.x, position.y, position.z))
            and control.position and not any((control.velocity, control.acceleration,
                                             control.attitude, control.body_rate)))


class StartupSelector:
    """Testable one-shot gate. A returned 'send' consumes its only attempt."""
    def __init__(self, now):
        self.deadline = now + 90
        self.stable_since = None
        self.sent_at = None
        self.finished = None

    def step(self, now, samples, receiver_ready):
        if self.finished is not None:
            return self.finished
        status, land = samples.get('status'), samples.get('land')
        if ((status and status[1].arming_state != 1)
                or (land and not land[1].landed)):
            self.finished = 'abort'
        elif (fresh(now, status, MAX_AGE['status'])
              and status[1].nav_state == 14):
            self.finished = 'confirmed'
        elif now >= self.deadline or (self.sent_at is not None and now-self.sent_at >= 5):
            self.finished = 'timeout'
        elif self.sent_at is None:
            if not eligible(now, samples) or not receiver_ready:
                self.stable_since = None
            elif self.stable_since is None:
                self.stable_since = now
            elif now-self.stable_since >= 2:
                self.sent_at = now
                return 'send'
        return self.finished


def mode_command(now_us):
    from px4_msgs.msg import VehicleCommand
    msg = VehicleCommand()
    msg.timestamp = now_us
    msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
    msg.param1, msg.param2 = 1.0, 6.0
    msg.target_system = msg.source_system = 12
    msg.target_component = 1
    msg.source_component = 198
    msg.from_external = True
    return msg


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import (VehicleStatus, VehicleLandDetected, VehicleLocalPosition,
                             OffboardControlMode, VehicleCommand)

    rclpy.init()
    node = rclpy.create_node('dashboard_startup_mode')
    samples = {}
    for topic, cls, key in (
        ('out/vehicle_status_v1', VehicleStatus, 'status'),
        ('out/vehicle_land_detected', VehicleLandDetected, 'land'),
        ('out/vehicle_local_position', VehicleLocalPosition, 'position'),
        ('in/offboard_control_mode', OffboardControlMode, 'control'),
    ):
        node.create_subscription(cls, '/claude/usb/'+topic,
            lambda msg, k=key: samples.update({k: (time.monotonic(), msg)}),
            qos_profile_sensor_data)
    pub = node.create_publisher(VehicleCommand, '/claude/usb/in/vehicle_command', 10)
    selector = StartupSelector(time.monotonic())
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            action = selector.step(time.monotonic(), samples, pub.get_subscription_count() > 0)
            if action == 'send':
                pub.publish(mode_command(node.get_clock().now().nanoseconds // 1000))
                node.get_logger().info('Requested Offboard once while disarmed and landed via USB')
            elif action == 'confirmed':
                node.get_logger().info('PX4 confirms Offboard; no arm command sent')
                return 0
            elif action == 'abort':
                node.get_logger().warn('No startup mode change: vehicle is armed or airborne')
                return 1
            elif action == 'timeout':
                node.get_logger().error('Offboard was not confirmed; no retry or arm command sent')
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

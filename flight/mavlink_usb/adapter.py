#!/usr/bin/env python3
"""Single-owner USB MAVLink flight transport and private return telemetry."""
from collections import Counter
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, ReliabilityPolicy,
                      DurabilityPolicy)
from px4_msgs.msg import (VehicleOdometry, OffboardControlMode, TrajectorySetpoint,
                         VehicleCommand, VehicleCommandAck, VehicleLocalPosition,
                         VehicleStatus, VehicleLandDetected, VehicleAttitude,
                         SensorCombined, BatteryStatus)
from std_msgs.msg import String

from conversion import odometry_message, odometry_metadata, SetpointPair, MAX_SAMPLE_AGE_US
from serial_router import SerialRouter
from return_telemetry import convert, command_message

ROOT = Path(__file__).resolve().parent


class Adapter(Node):
    def __init__(self):
        super().__init__('usb_mavlink_adapter')
        self.folder = ROOT/'runs'/time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
        self.folder.mkdir(parents=True,exist_ok=False)
        (ROOT/'active_run.txt').write_text(str(self.folder)+'\n')
        self.router = SerialRouter(
            '/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00',self.folder)
        self.pair = SetpointPair()
        self.counts = Counter()
        self.last_rejection = None
        return_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.return_publishers = {name: self.create_publisher(cls, '/claude/usb/out/'+name,
                                                           return_qos)
            for name, cls in (
                ('vehicle_local_position',VehicleLocalPosition),
                ('vehicle_status_v1',VehicleStatus),
                ('vehicle_land_detected',VehicleLandDetected),
                ('vehicle_attitude',VehicleAttitude), ('sensor_combined',SensorCombined),
                ('battery_status',BatteryStatus), ('vehicle_command_ack',VehicleCommandAck))}
        self.create_subscription(VehicleOdometry,'/claude/usb/in/vehicle_visual_odometry',
                                 self.vision,qos_profile_sensor_data)
        self.create_subscription(OffboardControlMode,'/claude/usb/in/offboard_control_mode',
                                 lambda msg:self.control('mode',msg),qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoint,'/claude/usb/in/trajectory_setpoint',
                                 lambda msg:self.control('setpoint',msg),qos_profile_sensor_data)
        self.create_subscription(VehicleCommand,'/claude/usb/in/vehicle_command',
                                 self.command,qos_profile_sensor_data)
        self.publisher = self.create_publisher(String,'/claude/usb/status',10)
        self.create_timer(1,self.status)
        self.create_timer(.02,self.return_data)
        self.router.start()
        self.get_logger().info('Vision, Offboard, explicit operator commands and return telemetry '
                               'use USB MAVLink. No automatic arm commands or command retries.')

    def return_data(self):
        if not self.router.connected:
            return
        for msg, received_at, wall_us in self.router.telemetry.take():
            try:
                result = convert(msg,wall_us)
                if result is not None:
                    topic, converted = result
                    self.return_publishers[topic].publish(converted)
                    self.counts['return_'+topic] += 1
            except (ValueError, TypeError, OverflowError) as exc:
                self.counts['return_rejected'] += 1
                self.last_rejection = str(exc)

    def command(self,msg):
        self.counts['command_received'] += 1
        try:
            now_us = time.time_ns()//1000
            packet = command_message(msg,now_us)
            if not self.router.connected:
                raise ValueError('USB disconnected; command not queued')
            ttl = min(.25,max(0,(250_000-(now_us-msg.timestamp))/1e6))
            if not self.router.queue.put('command',packet,ttl):
                raise ValueError('USB command queue full')
        except ValueError as exc:
            self.counts['command_rejected'] += 1
            self.last_rejection = str(exc)

    def vision(self,msg):
        self.counts['vision_received'] += 1
        now_us = time.time_ns()//1000
        try:
            packet = odometry_message(msg,now_us)
            ttl = min(.1, max(0,(MAX_SAMPLE_AGE_US-(now_us-msg.timestamp_sample))/1e6))
            self.router.queue.put('vision',packet,ttl,odometry_metadata(msg))
        except ValueError as exc:
            self.counts['vision_rejected'] += 1
            self.last_rejection = str(exc)

    def control(self,kind,msg):
        self.counts[kind+'_received'] += 1
        now_us = time.time_ns()//1000
        try:
            packet = self.pair.update(kind,msg,now_us)
            if packet is not None:
                ttl = min(.15,max(0,(250_000-(now_us-msg.timestamp))/1e6))
                self.router.queue.put('setpoint',packet,ttl,dict(publish_us=int(msg.timestamp),
                    type_mask=int(packet.type_mask),position=[packet.x,packet.y,packet.z],
                    velocity=[packet.vx,packet.vy,packet.vz]))
        except ValueError as exc:
            self.counts['control_rejected'] += 1
            self.last_rejection = str(exc)

    def status(self):
        state = dict(wall_ns=time.time_ns(),router_alive=self.router.is_alive(),
                     **self.router.snapshot(),input_counts=dict(self.counts),
                     last_rejection=self.last_rejection,run_dir=str(self.folder))
        data = json.dumps(state)
        temp = ROOT/'runtime-status.tmp'
        temp.write_text(data+'\n')
        temp.replace(ROOT/'runtime-status.json')
        self.publisher.publish(String(data=data))
        if not state['router_alive']:
            raise RuntimeError('USB router stopped: '+str(state['error']))


def main():
    rclpy.init()
    node = None
    try:
        node = Adapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.router.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()

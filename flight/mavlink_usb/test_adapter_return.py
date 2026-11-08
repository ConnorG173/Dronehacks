"""Real ROS + virtual serial integration; never opens flight hardware."""
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import rclpy
from rclpy.qos import qos_profile_sensor_data
from pymavlink.dialects.v20 import common as mav
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleCommandAck
import adapter
from serial_router import SerialRouter
from test_return_telemetry import odometry


class AdapterReturnTests(unittest.TestCase):
    def test_usb_decode_ros_delivery_and_no_stale_republication(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'),'221','Use isolated test domain')
        master,slave=os.openpty()
        device=os.ttyname(slave)
        os.close(slave)
        rclpy.init()
        node=None
        with tempfile.TemporaryDirectory() as directory:
            def factory(_real_device, folder):
                self.assertTrue(device.startswith('/dev/pts/'))
                return SerialRouter(device,folder,qgc_bind=('127.0.0.1',0),
                                    diag_bind=('127.0.0.1',0))
            with patch.object(adapter,'ROOT',Path(directory)), patch.object(adapter,'SerialRouter',factory):
                try:
                    node=adapter.Adapter()
                    rows={'position':[], 'status':[], 'ack':[]}
                    for topic,cls,key in (
                        ('vehicle_local_position',VehicleLocalPosition,'position'),
                        ('vehicle_status_v1',VehicleStatus,'status'),
                        ('vehicle_command_ack',VehicleCommandAck,'ack')):
                        node.create_subscription(cls,'/claude/usb/out/'+topic,
                            lambda m,k=key:rows[k].append(m),qos_profile_sensor_data)
                    deadline=time.monotonic()+2
                    while not node.router.connected and time.monotonic()<deadline:
                        rclpy.spin_once(node,timeout_sec=.02)
                    self.assertTrue(node.router.connected,node.router.error)
                    encoder=mav.MAVLink(None,srcSystem=12,srcComponent=1)
                    packets=[odometry(), mav.MAVLink_heartbeat_message(2,12,1,6<<16,3,3),
                        mav.MAVLink_command_ack_message(400,0,0,0,12,197)]
                    os.write(master,b''.join(m.pack(encoder) for m in packets))
                    deadline=time.monotonic()+2
                    while not all(rows.values()) and time.monotonic()<deadline:
                        rclpy.spin_once(node,timeout_sec=.02)
                    self.assertTrue(all(rows.values()),{k:len(v) for k,v in rows.items()})
                    self.assertEqual((rows['position'][-1].x,rows['position'][-1].z),(1.25,-.75))
                    self.assertEqual((rows['status'][-1].nav_state,rows['status'][-1].arming_state),(14,1))
                    self.assertEqual(rows['ack'][-1].target_component,197)
                    os.write(master,odometry().pack(encoder))  # frozen FC sample
                    deadline=time.monotonic()+.4
                    while time.monotonic()<deadline:
                        rclpy.spin_once(node,timeout_sec=.02)
                    self.assertEqual(len(rows['position']),1)
                finally:
                    if node:
                        node.router.close()
                        node.destroy_node()
                    rclpy.shutdown()
                    os.close(master)


if __name__=='__main__':unittest.main()

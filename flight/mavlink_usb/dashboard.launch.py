import os
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root='/root/team_ws/claude'
    limits=json.loads(Path(root+'/dash/flight_limits.json').read_text())
    return LaunchDescription([
        Node(package='skybeta_flight_interface',executable='flight_interface',
             name='flight_interface',output='screen',
             additional_env={'PYTHONPATH':root+'/height_stage:'+os.environ.get('PYTHONPATH','')},
             parameters=[{'target_system':12,'auto_offboard_on_arm':False,
                          'odometry_topic':'/claude/vio_leveled/odometry',
                          'use_px4_height':True,**limits}],
             remappings=[('/fmu/in/offboard_control_mode','/claude/usb/in/offboard_control_mode'),
                         ('/fmu/in/trajectory_setpoint','/claude/usb/in/trajectory_setpoint'),
                         ('/fmu/in/vehicle_command','/claude/usb/in/vehicle_command'),
                         ('/fmu/out/vehicle_status_v1','/claude/usb/out/vehicle_status_v1'),
                         ('/fmu/out/vehicle_local_position','/claude/usb/out/vehicle_local_position'),
                         ('/fmu/out/vehicle_land_detected','/claude/usb/out/vehicle_land_detected')]),
        ExecuteProcess(cmd=['python3',root+'/dash/dash_node.py','--bind','100.64.16.93',
                            '--port','8091','--live','--usb-telemetry',
                            '--min-alt',str(limits['min_altitude_m']),
                            '--max-alt',str(limits['max_altitude_m']),
                            '--horizontal-speed',str(limits['max_speed_mps'])],cwd=root+'/dash',
                       output='screen',respawn=True,respawn_delay=3.0),
        ExecuteProcess(cmd=['python3','-u',root+'/mavlink_usb/adapter.py'],
                       cwd=root+'/mavlink_usb',output='screen',
                       on_exit=Shutdown(reason='USB MAVLink adapter exited')),
        # Passive display worker: its exit/restart does not stop flight nodes.
        ExecuteProcess(cmd=['nice','-n','10','python3','-u',
                            root+'/depth_overlay/start_when_grounded.py','--hz','2'],
                       cwd=root+'/depth_overlay',output='screen',
                       respawn=True,respawn_delay=10.0),
        # Preserve the one-shot grounded mode selector, using the USB return feed.
        ExecuteProcess(cmd=['python3',root+'/mavlink_usb/select_offboard_usb.py'],
                       output='screen'),
    ])

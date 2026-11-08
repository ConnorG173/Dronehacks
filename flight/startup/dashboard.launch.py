import os
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root = '/root/team_ws/claude'
    limits = json.loads(Path(root + '/dash/flight_limits.json').read_text())
    return LaunchDescription([
        Node(
            package='skybeta_flight_interface', executable='flight_interface',
            name='flight_interface', output='screen',
            additional_env={'PYTHONPATH': root + '/height_stage:'
                            + os.environ.get('PYTHONPATH', '')},
            parameters=[{'target_system': 12, 'auto_offboard_on_arm': False,
                         'odometry_topic': '/claude/vio_leveled/odometry',
                         'use_px4_height': True, **limits}]),
        ExecuteProcess(
            cmd=['python3', root + '/dash/dash_node.py', '--bind',
                 '100.64.16.93', '--port', '8091', '--live',
                 '--min-alt', str(limits['min_altitude_m']),
                 '--max-alt', str(limits['max_altitude_m']),
                 '--horizontal-speed', str(limits['max_speed_mps'])],
            cwd=root + '/dash', output='screen', respawn=True, respawn_delay=3.0),
        ExecuteProcess(
            cmd=['python3', root + '/mav_udp.py', '--laptop', '100.107.69.115'],
            cwd=root, output='screen', respawn=True, respawn_delay=3.0),
        ExecuteProcess(
            cmd=['python3', root + '/startup/select_offboard.py'],
            output='screen'),
    ])

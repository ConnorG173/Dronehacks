"""Real drone (skybeta Jetson): the follower's real adapter (+ flight_interface, exactly as
competitor.launch.py starts it).

  # next to the stack's running flight_interface: preview topic only (the defaults)
  ros2 launch localizer real.launch.py

  # the bench step (team approval): our lead points reach flight_interface -> PX4
  ros2 launch localizer real.launch.py live:=true

with_flight_interface:=true also starts flight_interface exactly as competitor.launch.py does. Use
it only where nothing else runs one, e.g. when the stack starts this launch as
COMPETITOR_PKG=localizer / COMPETITOR_LAUNCH=real.launch.py (ask first: /etc/skybeta/drone.env).
Never start a second flight_interface next to a running one.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('localizer')
    args = [
        DeclareLaunchArgument('with_flight_interface', default_value='false'),
        DeclareLaunchArgument('live', default_value='false',
                              description='true: publish to /skybeta/setpoint/pose; '
                                          'false: /localizer/setpoint_preview only'),
        DeclareLaunchArgument('path_file', default_value=os.path.join(share, 'data', 'course_v3_path.bin')),
        DeclareLaunchArgument('max_path_speed_mps', default_value='1.0'),
        DeclareLaunchArgument('command_yaw', default_value='true'),
        DeclareLaunchArgument('mpc_xy_p', default_value='0.95'),
        DeclareLaunchArgument('dry_run', default_value='false'),
    ]
    flight_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('competitor_example'), 'launch', 'competitor.launch.py')),
        launch_arguments={'competitor': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('with_flight_interface')),
    )
    follower = Node(
        package='localizer', executable='real_follower', name='real_follower', output='screen',
        emulate_tty=True,
        parameters=[{
            'path_file': LaunchConfiguration('path_file'),
            'live': ParameterValue(LaunchConfiguration('live'), value_type=bool),
            'max_path_speed_mps': ParameterValue(LaunchConfiguration('max_path_speed_mps'), value_type=float),
            'command_yaw': ParameterValue(LaunchConfiguration('command_yaw'), value_type=bool),
            'mpc_xy_p': ParameterValue(LaunchConfiguration('mpc_xy_p'), value_type=float),
            'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
        }],
    )
    return LaunchDescription(args + [flight_interface, follower])

"""localizer: path_streamer + course_matcher (+ flight_interface, exactly as competitor.launch.py).

  # alongside an already-running flight_interface (the bench test): preview topic only
  ros2 launch localizer localizer.launch.py with_flight_interface:=false

  # the bench step: our setpoints reach flight_interface (and so /fmu/in/trajectory_setpoint)
  ros2 launch localizer localizer.launch.py with_flight_interface:=false live:=true

As COMPETITOR_PKG=localizer (ask first: /etc/skybeta/drone.env), with_flight_interface
defaults true so the stack's competitor window gets the flight interface it needs.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('localizer')
    args = [
        DeclareLaunchArgument('with_flight_interface', default_value='true'),
        DeclareLaunchArgument('live', default_value='false',
                              description='true: publish to /skybeta/setpoint/pose; '
                                          'false: /localizer/setpoint_preview only'),
        DeclareLaunchArgument('matcher', default_value='true'),
        DeclareLaunchArgument('path_file', default_value=os.path.join(share, 'data', 'course_v3_path.bin')),
        DeclareLaunchArgument('grid_file', default_value=os.path.join(share, 'data', 'course_v3_wallgrid.npz')),
        DeclareLaunchArgument('max_path_speed_mps', default_value='1.0'),
    ]
    flight_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('competitor_example'), 'launch', 'competitor.launch.py')),
        launch_arguments={'competitor': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('with_flight_interface')),
    )
    setpoint_topic = PythonExpression([
        "'/skybeta/setpoint/pose' if '", LaunchConfiguration('live'),
        "'.lower() == 'true' else '/localizer/setpoint_preview'"])
    streamer = Node(
        package='localizer', executable='path_streamer', name='path_streamer', output='screen',
        emulate_tty=True,
        parameters=[{
            'path_file': LaunchConfiguration('path_file'),
            'grid_file': LaunchConfiguration('grid_file'),
            'setpoint_topic': setpoint_topic,
            'max_path_speed_mps': LaunchConfiguration('max_path_speed_mps'),
        }],
    )
    matcher = Node(
        package='localizer', executable='course_matcher', name='course_matcher', output='screen',
        emulate_tty=True, condition=IfCondition(LaunchConfiguration('matcher')),
        parameters=[{'grid_file': LaunchConfiguration('grid_file')}],
    )
    return LaunchDescription(args + [flight_interface, streamer, matcher])

"""Localization for drone 12: loc_fuser (milestone 1). Include this from the team's competitor launch.

    ros2 launch team_loc team_loc.launch.py course_config:=/abs/path/course_v3.yaml base_height_on_pad_m:=0.10

`course_config` (required) is the planner config the path was planned from (StartPos,
StartHeadingDeg), e.g. <repo>/planner/config/course_v3.yaml. `base_height_on_pad_m` is a vehicle
value: the Pixhawk IMU's height above the floor with the drone on the pad.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('team_loc')
    args = [
        DeclareLaunchArgument('course_config', description='planner course config (StartPos, StartHeadingDeg)'),
        DeclareLaunchArgument('base_height_on_pad_m', default_value='0.0',
                              description='base_link (Pixhawk IMU) height above the floor on the pad'),
        DeclareLaunchArgument('enable_correction', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    fuser = Node(
        package='team_loc', executable='loc_fuser', name='loc_fuser', output='screen', emulate_tty=True,
        parameters=[
            os.path.join(share, 'config', 'loc_fuser.yaml'),
            {
                'course_config': LaunchConfiguration('course_config'),
                'base_height_on_pad_m': LaunchConfiguration('base_height_on_pad_m'),
                'enable_correction': LaunchConfiguration('enable_correction'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )
    return LaunchDescription(args + [fuser])

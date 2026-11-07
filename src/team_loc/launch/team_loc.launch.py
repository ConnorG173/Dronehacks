"""Localization for drone 12: loc_fuser (+ wall_matcher). Include this from the team's competitor launch.

    ros2 launch team_loc team_loc.launch.py course_config:=<repo>/planner/config/course_v3.yaml \
        base_height_on_pad_m:=0.10 [enable_correction:=true with_matcher:=true]

`course_config` (required): the planner config the path was planned from (StartPos, StartHeadingDeg,
MapFile). `base_height_on_pad_m`: a vehicle value, the Pixhawk IMU's height above the floor with the
drone on the pad.

Defaults are milestone 1 (anchor + republish, no correction, no matcher), the safe choice until the
correction has passed its gate (docs/proposals/localization.md section 6). `enable_correction:=true`
without the matcher is harmless: no fixes arrive, so the correction stays at zero.
`matcher_source:=depth` takes a depth image on /team/loc/depth instead of the stereo pair.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('team_loc')
    L = LaunchConfiguration
    args = [
        DeclareLaunchArgument('course_config', description='planner course config (StartPos, StartHeadingDeg, MapFile)'),
        DeclareLaunchArgument('base_height_on_pad_m', default_value='0.0',
                              description='base_link (Pixhawk IMU) height above the floor on the pad'),
        DeclareLaunchArgument('enable_correction', default_value='false'),
        DeclareLaunchArgument('with_matcher', default_value='false'),
        DeclareLaunchArgument('matcher_source', default_value='stereo', description='stereo | depth'),
        DeclareLaunchArgument('matcher_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('left_image_topic', default_value='/oak/left/image_mono'),
        DeclareLaunchArgument('right_image_topic', default_value='/oak/right/image_mono'),
        DeclareLaunchArgument('left_info_topic', default_value='/oak/left/camera_info'),
        DeclareLaunchArgument('right_info_topic', default_value='/oak/right/camera_info'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    fuser = Node(
        package='team_loc', executable='loc_fuser', name='loc_fuser', output='screen', emulate_tty=True,
        parameters=[
            os.path.join(share, 'config', 'loc_fuser.yaml'),
            {
                'course_config': L('course_config'),
                'base_height_on_pad_m': L('base_height_on_pad_m'),
                'enable_correction': L('enable_correction'),
                'use_sim_time': L('use_sim_time'),
            },
        ],
    )
    matcher = Node(
        package='team_loc_matcher', executable='wall_matcher', name='wall_matcher', output='screen',
        emulate_tty=True, condition=IfCondition(L('with_matcher')),
        # Below the stack and the control loop: never starve cuVSLAM or the flight interface.
        prefix='nice -n 10',
        parameters=[{
            'course_config': L('course_config'),
            'source': L('matcher_source'),
            'rate_hz': L('matcher_rate_hz'),
            'left_image_topic': L('left_image_topic'),
            'right_image_topic': L('right_image_topic'),
            'left_info_topic': L('left_info_topic'),
            'right_info_topic': L('right_info_topic'),
            'use_sim_time': L('use_sim_time'),
        }],
    )
    return LaunchDescription(args + [fuser, matcher])

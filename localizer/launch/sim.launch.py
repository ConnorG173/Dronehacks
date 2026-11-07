"""Simulator: the follower's sim adapter, for a colcon-installed workspace.

  ros2 launch localizer sim.launch.py                    # passive: no publishers, nothing is sent
  ros2 launch localizer sim.launch.py approved:=true     # arms and flies (team approval, Reset world)

The lab container normally runs it without colcon:
  cd localizer && python3 -m localizer.sim [--i-have-team-approval]
which also picks up the deployment parameter file (/etc/flyx/mission.yaml: use_sim_time).
Here, pass use_sim_time:=true yourself if that file is not used.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def follower(context):
    approved = LaunchConfiguration('approved').perform(context).lower() == 'true'
    return [Node(
        package='localizer', executable='sim_follower', name='sim_follower', output='screen',
        emulate_tty=True,
        arguments=['--i-have-team-approval'] if approved else [],
        parameters=[{
            'path_file': LaunchConfiguration('path_file').perform(context),
            'max_path_speed_mps': float(LaunchConfiguration('max_path_speed_mps').perform(context)),
            'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
        }],
    )]


def generate_launch_description():
    share = get_package_share_directory('localizer')
    return LaunchDescription([
        DeclareLaunchArgument('approved', default_value='false',
                              description='true only after the team approved the mission'),
        DeclareLaunchArgument('path_file', default_value=os.path.join(share, 'data', 'course_v3_path.bin')),
        DeclareLaunchArgument('max_path_speed_mps', default_value='1.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=follower),
    ])

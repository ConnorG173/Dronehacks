from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '/root/team_ws/claude/vio_level.py'],
            cwd='/root/team_ws/claude', output='screen',
            on_exit=Shutdown(reason='VIO leveller exited')),
        Node(
            package='oakd_vslam_bringup', executable='px4_odom_bridge',
            name='px4_odom_bridge', output='screen',
            parameters=[{'input_topic': '/claude/vio_leveled/odometry',
                         'pose_frame': 'FRD'}],
            on_exit=Shutdown(reason='PX4 odometry bridge exited')),
    ])

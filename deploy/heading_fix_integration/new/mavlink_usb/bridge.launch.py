import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown
from launch_ros.actions import Node

# Gyro heading fix (team, branch imu-fusion, deploy/imu_fusion_shadow/heading_fix.py). Switched by a marker:
#   touch /root/team_ws/claude/mavlink_usb/heading_fix_enabled   -> PX4 gets /team_heading/odometry
#   rm    /root/team_ws/claude/mavlink_usb/heading_fix_enabled   -> exactly the previous chain (levelled VIO)
# (restart this launch / reboot to apply). The node takes heading increments from PX4's gyro and everything
# else from the levelled VIO; the dashboard blocks ARM until it has >= 10 s of still gyro data.
HEADING_FIX = '/root/team_ws/team_staging/imu_fusion/heading_fix.py'
ENABLED = os.path.exists('/root/team_ws/claude/mavlink_usb/heading_fix_enabled') and os.path.exists(HEADING_FIX)


def generate_launch_description():
    actions = [
        ExecuteProcess(cmd=['python3','/root/team_ws/claude/vio_level.py','--usb-telemetry'],
                       cwd='/root/team_ws/claude',output='screen',
                       on_exit=Shutdown(reason='VIO leveller exited')),
    ]
    if ENABLED:
        actions.append(ExecuteProcess(
            cmd=['python3', '-u', HEADING_FIX, 'node', '--gyro-topic', '/fmu/out/sensor_combined'],
            cwd=os.path.dirname(HEADING_FIX), output='screen',
            on_exit=Shutdown(reason='heading fix exited')))
    actions.append(
        Node(package='oakd_vslam_bringup',executable='px4_odom_bridge',
             name='px4_odom_bridge',output='screen',
             parameters=[{'input_topic':'/team_heading/odometry' if ENABLED else '/claude/vio_leveled/odometry',
                          'pose_frame':'FRD',
                          'output_topic':'/claude/usb/in/vehicle_visual_odometry'}],
             on_exit=Shutdown(reason='PX4 odometry bridge exited')))
    return LaunchDescription(actions)

"""Frozen drone profile "conservative" (2026-11-07 night runs): mission.launch.py with the settings
validated for tonight, which tuning work must not change:

    Gate B route (planner/config/course_v3_gateb.yaml), plan MaxVelMag 1.2 slowed 1.4x (peak ~0.86 m/s),
    tracker gains config/tracker_conservative.yaml (lag tau_v 0.66 / t_delay 0.18), path clock <= 1.0,
    flight interface as competitor.launch.py (1.5 m/s, 10 Hz, auto Offboard on arm, no mode requests).

Only these pass through (everything else is fixed here):
    mission_preset (hover | first_leg | course; default hover), pose_topic, pose_topic_frame,
    base_height_on_pad_m, repo_root, planner_binary, out_dir, allow_mode_request (sim only, default false).

    ros2 launch team_path_following conservative.launch.py mission_preset:=course \
        pose_topic:=/visual_slam/tracking/odometry pose_topic_frame:=odom base_height_on_pad_m:=0.08
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))   # <repo> when run from the source tree

FIXED = {
    'time_scale': '1.4',
    'speed_factor': '1.0',
    'planner_overrides': 'MaxVelMag=1.2',
    'max_speed_mps': '1.5',
    'setpoint_rate_hz': '10.0',
    'auto_offboard_on_arm': 'true',
    'z_ceiling_m': '1.6',
    'return_config': '',
}
PASS = [
    ('mission_preset', 'hover', 'hover | first_leg | course (default hover: a restart never flies the course)'),
    ('pose_topic', '/team/odom_map', 'course-frame pose; raw VIO: /visual_slam/tracking/odometry'),
    ('pose_topic_frame', 'course', 'course | odom (odom = raw VIO + launch-pose fallback)'),
    ('base_height_on_pad_m', '0.0', 'base_link height above the floor on the pad'),
    ('repo_root', ROOT, 'repository root (planner, course config, map)'),
    ('planner_binary', '', "default <repo_root>/planner/build/gcopter_planner"),
    ('out_dir', '/tmp/team_path_following', 'planner output directory'),
    ('allow_mode_request', 'false', 'SIM ONLY (Gazebo pilot.py): let the interface accept mode requests'),
]


def _include(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    root = lc('repo_root')
    try:
        params = os.path.join(get_package_share_directory('team_path_following'), 'config', 'tracker_conservative.yaml')
    except Exception:  # noqa: BLE001
        params = os.path.join(HERE, '..', 'config', 'tracker_conservative.yaml')
    args = dict(FIXED)
    args.update({n: lc(n) for n, _, _ in PASS})
    args['course_config'] = os.path.join(root, 'planner', 'config', 'course_v3_gateb.yaml')
    args['planner_binary'] = args['planner_binary'] or os.path.join(root, 'planner', 'build', 'gcopter_planner')
    args['tracker_params'] = params
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(HERE, 'mission.launch.py')),
                                     launch_arguments=list(args.items()))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in PASS]
                             + [OpaqueFunction(function=_include)])

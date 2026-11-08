"""Flight interface + mission_tracker.

The flight_interface node is declared exactly as in the organizers'
competitor_example/launch/competitor.launch.py (same arguments, same parameters),
as their guide requires. Only mission_tracker is ours.

    ros2 launch team_path_following mission.launch.py
    ros2 launch team_path_following mission.launch.py course_config:=/path/to/course.yaml \
        planner_overrides:="MaxVelMag=1.2" setpoint_rate_hz:=10.0
    # sim only: allow_mode_request:=true

Build with --symlink-install (team_build.sh does) so the repo root is found from this
file's real location; otherwise pass repo_root:=...
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _repo_root():
    # <repo>/path_following/team_path_following/launch/mission.launch.py
    return os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..'))


# flight interface: name -> (default, help), identical to competitor.launch.py
FI_ARGUMENTS = [
    ('max_altitude_m', '3.0', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0', 'OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz'),
    ('allow_mode_request', 'false', 'enable /skybeta/request_offboard. SIM ONLY'),
    ('auto_offboard_on_arm', 'true', 'enter Offboard once when the pilot arms, then switches to Position'),
    ('odometry_topic', '/visual_slam/tracking/odometry', 'source of the ENU pose the altitude band is measured against'),
]

ROOT = _repo_root()
OUR_ARGUMENTS = [
    ('repo_root', ROOT, 'repository root (planner binary, course config)'),
    ('course_config', os.path.join(ROOT, 'planner', 'config', 'course_v3_gateb.yaml'),
     'planner course config (Gate B route; course_v3.yaml = the window route)'),
    ('planner_binary', os.path.join(ROOT, 'planner', 'build', 'gcopter_planner'), 'GCOPTER planner executable'),
    ('planner_overrides', 'MaxVelMag=1.2', 'config overrides, "Key=value@Key=value" (or ;)'),
    ('path_file', '', 'fly this DHPATH file instead of planning'),
    ('tracker_params', os.path.join(ROOT, 'path_following', 'team_path_following', 'config', 'tracker.yaml'),
     'mission_tracker parameter file'),
    ('pose_topic', '/team/odom_map', 'course-frame pose (localization); sim ground truth: /sim/ground_truth/odom'),
    ('pose_topic_frame', 'course', 'frame of pose_topic: course | odom'),
    ('base_height_on_pad_m', '0.0', 'base_link height above the floor on the pad (fallback map; same name as team_loc)'),
    ('time_scale', '1.0', 'uniform slow-down (>1) of the loaded plan'),
    ('mission_preset', 'hover', 'hover (default: take off, hold, land) | first_leg (slowly to checkpoint 1, hold, land there) | '
     'course; default hover so a stack restart never flies the course by surprise'),
    ('hover_hold_s', '10.0', 'hover preset: hold time at HoverHeight'),
    ('first_leg_speed_mps', '0.4', 'first_leg preset: planner MaxVelMag'),
    ('first_leg_hold_s', '5.0', 'first_leg preset: hold at checkpoint 1 before landing'),
    ('speed_factor', '1.0', 'max path-clock rate: >1 flies faster than planned (1.15 validated in kinsim)'),
    ('z_ceiling_m', '1.6', 'map z ceiling enforced by mission_tracker'),
    ('return_config', '', "return leg: '' off | 'reverse' (goal -> start) | config path (needs organizers' OK)"),
    ('return_z_ceiling_m', '1.6', 'map z ceiling on the return leg'),
    ('out_dir', '/tmp/team_path_following', 'planner output directory'),
]


def _nodes(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_altitude_m': LaunchConfiguration('max_altitude_m'),
            'min_altitude_m': LaunchConfiguration('min_altitude_m'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
            'setpoint_rate_hz': LaunchConfiguration('setpoint_rate_hz'),
            'allow_mode_request': LaunchConfiguration('allow_mode_request'),
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
        }],
    )
    params = []
    if lc('tracker_params') and os.path.exists(lc('tracker_params')):
        params.append(lc('tracker_params'))
    params.append({
        'course_config': lc('course_config'),
        'planner_binary': lc('planner_binary'),
        'planner_overrides': lc('planner_overrides'),
        'path_file': lc('path_file'),
        'pose_topic': lc('pose_topic'),
        'pose_topic_frame': lc('pose_topic_frame'),
        'base_height_on_pad_m': float(lc('base_height_on_pad_m')),
        'time_scale': float(lc('time_scale')),
        'mission_preset': lc('mission_preset'),
        'hover_hold_s': float(lc('hover_hold_s')),
        'first_leg_speed_mps': float(lc('first_leg_speed_mps')),
        'first_leg_hold_s': float(lc('first_leg_hold_s')),
        'tracker.taudot_max': float(lc('speed_factor')),
        'max_speed_mps': float(lc('max_speed_mps')),
        'z_ceiling_m': float(lc('z_ceiling_m')),
        'return_config': lc('return_config'),
        'return_z_ceiling_m': float(lc('return_z_ceiling_m')),
        'out_dir': lc('out_dir'),
    })
    tracker = Node(
        package='team_path_following',
        executable='mission_tracker',
        name='mission_tracker',
        output='screen',
        emulate_tty=True,
        parameters=params,
    )
    return [flight_interface, tracker]


def generate_launch_description():
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in FI_ARGUMENTS + OUR_ARGUMENTS]
    return LaunchDescription(declared + [OpaqueFunction(function=_nodes)])

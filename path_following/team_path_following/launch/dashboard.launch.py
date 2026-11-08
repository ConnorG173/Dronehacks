"""drone-conservative-v1 on drone 12's current framework (the teammates' dashboard stack).

Same flight as conservative.launch.py (Gate B route, plan MaxVelMag 1.2 slowed 1.4x, peak ~0.86 m/s,
tracker_conservative.yaml lag 0.66/0.18, path clock <= 1.0, z ceiling 1.6 m), adapted to how drone 12 runs now:

  * No flight interface of ours. The dashboard launch (competitor window) already runs the flight interface,
    remapped onto the USB MAVLink link that carries vision and setpoints to PX4. Starting a second one, or
    stopping that launch, breaks the vehicle's vision feed. mission_tracker only publishes
    /skybeta/setpoint/twist into the running interface.
  * Pose = /claude/vio_leveled/odometry, the leveled cuVSLAM that the interface and PX4 use (frame odom; the
    course frame comes from the pose at arming, so place the drone on the launch mark, nose +x, first).
  * dashboard_coexist: publishes nothing until the drone is armed in Offboard; captures the launch pose at
    arming; the moment the dashboard sends any pose setpoint or the interface reports Hold (frozen), it stops
    for good and publishes nothing, so the dashboard's Hold / Land / KILL keep working.
  * max_speed_mps 1.0 = the dashboard interface's limit (dash/flight_limits.json).

Only these pass through:
    mission_preset (hover | first_leg | course; default hover), base_height_on_pad_m, pose_topic,
    repo_root, planner_binary, out_dir, px4_local_position_topic.

    ros2 launch team_path_following dashboard.launch.py repo_root:=<staged repo> mission_preset:=hover \
        base_height_on_pad_m:=<measured>
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
    # drone-conservative-v1 flight settings (unchanged)
    'time_scale': '1.4',
    'speed_factor': '1.0',
    'planner_overrides': 'MaxVelMag=1.2',
    'z_ceiling_m': '1.6',
    'return_config': '',
    'pose_topic_frame': 'odom',
    # drone 12 framework
    'start_flight_interface': 'false',
    'dashboard_coexist': 'true',
    'max_speed_mps': '1.0',
    'allow_mode_request': 'false',
    'tracker_use_sim_time': 'false',
}
PASS = [
    ('mission_preset', 'hover', 'hover | first_leg | course (default hover: a restart never flies the course)'),
    ('base_height_on_pad_m', '0.0', 'base_link height above the floor on the pad'),
    ('pose_topic', '/claude/vio_leveled/odometry', 'leveled cuVSLAM used by the dashboard interface and PX4'),
    ('repo_root', ROOT, 'repository root (planner, course config, map)'),
    ('planner_binary', '', 'default <repo_root>/planner/build/gcopter_planner'),
    ('out_dir', '/tmp/team_path_following', 'planner output directory'),
    ('px4_local_position_topic', '/fmu/out/vehicle_local_position', 'PX4 local position (UART DDS return telemetry)'),
    ('path_file', '', "fly this DHPATH route (launch-relative NED) instead of planning Gate B; '' = plan"),
    ('peak_speed_mps', '0.86', 'with path_file: slow the route uniformly to this peak (never faster than the file)'),
]


def _route_time_scale(path_file, peak):
    """Uniform slow-down so the file's peak horizontal+vertical speed becomes `peak` (>= 1.0)."""
    import struct
    import numpy as np
    raw = open(path_file, 'rb').read()
    if raw[:6] != b'DHPATH':
        raise ValueError(f'{path_file} is not a DHPATH file')
    rows = struct.unpack_from('<Q', raw, 16)[0]
    a = np.frombuffer(raw, '<f8', rows * 12, 32).reshape(rows, 12)
    vmax = float(np.max(np.linalg.norm(a[:, 4:7], axis=1)))
    return max(1.0, vmax / float(peak)), vmax


def _include(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    root = lc('repo_root')
    try:
        params = os.path.join(get_package_share_directory('team_path_following'), 'config', 'tracker_conservative.yaml')
    except Exception:  # noqa: BLE001
        params = os.path.join(HERE, '..', 'config', 'tracker_conservative.yaml')
    args = dict(FIXED)
    args.update({n: lc(n) for n, _, _ in PASS})
    path_file, peak = args.pop('path_file'), args.pop('peak_speed_mps')
    if path_file:
        if args['mission_preset'] == 'first_leg':
            raise RuntimeError('first_leg re-plans on the CAD map; with path_file fly hover, then course')
        k, vmax = _route_time_scale(path_file, peak)
        args['path_file'] = path_file
        args['time_scale'] = f'{k:.3f}'
        print(f'[dashboard.launch] route {path_file}: file peak {vmax:.2f} m/s, time_scale {k:.2f} -> peak {vmax / k:.2f} m/s')
    args['vio_topic'] = args['pose_topic']      # no fallback to raw (unleveled) VIO
    args['course_config'] = os.path.join(root, 'planner', 'config', 'course_v3_gateb.yaml')
    args['planner_binary'] = args['planner_binary'] or os.path.join(root, 'planner', 'build', 'gcopter_planner')
    args['tracker_params'] = params
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(HERE, 'mission.launch.py')),
                                     launch_arguments=list(args.items()))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in PASS]
                             + [OpaqueFunction(function=_include)])

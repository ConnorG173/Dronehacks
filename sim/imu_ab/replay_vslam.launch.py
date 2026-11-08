# cuVSLAM alone, fed from a recorded sim bag (sim/imu_ab/replay.sh): the same VisualSlamNode parameters as
# the organizers' sim launch (sim/launch/skybeta_sim_imu.launch.py for fusion), without the Gazebo-side
# format converters, relays, uXRCE agent or PX4 bridge. The bag supplies /oak/{left,right}/image_mono
# (mono8, already converted in the flight container), camera_info, /drone1/sensors/imu and /clock.
# Same TF tree as the sim launch: base_link -> oak_parent_frame -> left/right, base_link -> sim_imu.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue

ARGS = (('enable_imu_fusion', 'False'), ('imu_topic', '/drone1/sensors/imu'), ('imu_frequency', '250.0'),
        ('gyro_noise_density', '0.000244'), ('gyro_random_walk', '0.000019393'), ('accel_noise_density', '0.001862'),
        ('accel_random_walk', '0.003'), ('imu_roll', '0.0'), ('imu_pitch', '0.0'), ('imu_yaw', '0.0'),
        ('imu_x', '0.0'), ('imu_y', '0.0'), ('imu_z', '0.0'),
        ('camera_x', '0.12'), ('camera_z', '0.242'), ('baseline', '0.075'), ('image_jitter_threshold_ms', '40.0'),
        ('imu_jitter_threshold_ms', '10.0'), ('imu_qos_depth', '0'), ('verbosity', '1'), ('sync_matching_threshold_ms', '5.0'))


def generate_launch_description():
    L = LaunchConfiguration
    f = lambda n: ParameterValue(L(n), value_type=float)  # noqa: E731
    tfs = [
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_rig',
             arguments=['--x', L('camera_x'), '--z', L('camera_z'), '--frame-id', 'base_link',
                        '--child-frame-id', 'oak_parent_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='rig_to_left',
             arguments=['--y', PythonExpression(['0.5 * ', L('baseline')]), '--roll', '-1.5707963267948966',
                        '--yaw', '-1.5707963267948966', '--frame-id', 'oak_parent_frame', '--child-frame-id', 'left']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='rig_to_right',
             arguments=['--y', PythonExpression(['-0.5 * ', L('baseline')]), '--roll', '-1.5707963267948966',
                        '--yaw', '-1.5707963267948966', '--frame-id', 'oak_parent_frame', '--child-frame-id', 'right']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_imu',
             arguments=['--x', L('imu_x'), '--y', L('imu_y'), '--z', L('imu_z'), '--roll', L('imu_roll'),
                        '--pitch', L('imu_pitch'), '--yaw', L('imu_yaw'), '--frame-id', 'base_link',
                        '--child-frame-id', 'sim_imu']),
    ]
    vslam = ComposableNode(
        package='isaac_ros_visual_slam', plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'use_sim_time': True,
            'enable_image_denoising': False, 'rectified_images': True,
            'enable_imu_fusion': ParameterValue(L('enable_imu_fusion'), value_type=bool),
            'imu_frame': 'sim_imu', 'calibration_frequency': f('imu_frequency'),
            'gyro_noise_density': f('gyro_noise_density'), 'gyro_random_walk': f('gyro_random_walk'),
            'accel_noise_density': f('accel_noise_density'), 'accel_random_walk': f('accel_random_walk'),
            'imu_jitter_threshold_ms': f('imu_jitter_threshold_ms'),
            'imu_qos_depth': ParameterValue(L('imu_qos_depth'), value_type=int),
            'verbosity': ParameterValue(L('verbosity'), value_type=int),
            'sync_matching_threshold_ms': f('sync_matching_threshold_ms'),
            'base_frame': 'base_link', 'map_frame': 'map', 'odom_frame': 'odom',
            'camera_optical_frames': ['left', 'right'],
            'publish_odom_to_base_tf': False, 'publish_map_to_odom_tf': False,
            'image_jitter_threshold_ms': f('image_jitter_threshold_ms'),
            'enable_localization_n_mapping': False, 'enable_slam_visualization': False,
            'enable_landmarks_view': False, 'enable_observations_view': False,
        }],
        remappings=[
            ('visual_slam/image_0', '/oak/left/image_mono'),
            ('visual_slam/camera_info_0', '/drone1/left/color/camera_info'),
            ('visual_slam/image_1', '/oak/right/image_mono'),
            ('visual_slam/camera_info_1', '/drone1/right/color/camera_info'),
            ('visual_slam/imu', L('imu_topic')),
        ],
    )
    container = ComposableNodeContainer(
        name='vslam_container', namespace='', package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[vslam], output='screen', parameters=[{'use_sim_time': True}])
    return LaunchDescription([DeclareLaunchArgument(n, default_value=v) for n, v in ARGS] + tfs + [container])

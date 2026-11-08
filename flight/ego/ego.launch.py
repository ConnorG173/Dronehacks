"""EGO-Planner for skybeta-12 (single drone), PX4-local ENU frame ('world') shared with the dashboard.

Inputs  /ego/odom (ego_bridge.py), /ego/depth + /ego/camera_pose (depth_overlay worker, same stamps),
        /move_base_simple/goal (dashboard, carrot goal along the course route)
Output  /position_cmd -> ego_bridge.py -> /ego/setpoint (JSON) -> dashboard in 'ego' obstacle mode
Intrinsics are read from ../depth_overlay/ego_camera.json, written by the depth worker.
"""
import json
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
CAM = os.path.join(HERE, '..', 'depth_overlay', 'ego_camera.json')

# ---- calibration knobs -----------------------------------------------------------------------
MAX_VEL, MAX_ACC = .8, 1.5          # course speed cap is 1 m/s; stay below it
HORIZON = 3.0                        # depth is trusted to ~3.5-4 m
RESOLUTION, INFLATION = .08, .08     # 8 cm cells like our depth grid; inflation = one cell
DIST0 = .25                          # centre-to-edge clearance = DIST0 + INFLATION = 0.33 m (drone half-width 0.26): openings >= ~0.66 m
CEIL, GROUND = 3.5, -1.5          # PX4-local z, whose zero is not the floor (seen -0.44 m on the ground); the dashboard's floor/ceiling still clamp


def generate_launch_description():
    with open(CAM) as f:
        cam = json.load(f)                       # {"fx", "fy", "cx", "cy", "width", "height"}
    planner = Node(
        package='ego_planner', executable='ego_planner_node', name='ego_planner_node', output='screen',
        remappings=[('odom_world', '/ego/odom'), ('grid_map/odom', '/ego/odom'),
                    ('grid_map/depth', '/ego/depth'), ('grid_map/pose', '/ego/camera_pose'),
                    ('grid_map/cloud', '/ego/cloud_unused'),
                    ('planning/broadcast_bspline_from_planner', '/ego/broadcast_bspline'),
                    ('planning/broadcast_bspline_to_planner', '/ego/broadcast_bspline'),
                    ('grid_map/occupancy_inflate', '/ego/occupancy_inflate')],
        parameters=[{
            'fsm/flight_type': 1, 'fsm/thresh_replan_time': 1.0, 'fsm/thresh_no_replan_meter': .5,
            'fsm/planning_horizon': HORIZON, 'fsm/planning_horizen_time': 3.0, 'fsm/emergency_time': 1.0,
            'fsm/realworld_experiment': False, 'fsm/fail_safe': True, 'fsm/waypoint_num': 0,
            'grid_map/resolution': RESOLUTION, 'grid_map/map_size_x': 20.0, 'grid_map/map_size_y': 20.0,
            'grid_map/map_size_z': 5.0, 'grid_map/local_update_range_x': 4.0,
            'grid_map/local_update_range_y': 4.0, 'grid_map/local_update_range_z': 2.5,
            'grid_map/obstacles_inflation': INFLATION, 'grid_map/local_map_margin': 10,
            'grid_map/ground_height': GROUND,
            'grid_map/cx': float(cam['cx']), 'grid_map/cy': float(cam['cy']),
            'grid_map/fx': float(cam['fx']), 'grid_map/fy': float(cam['fy']),
            'grid_map/use_depth_filter': True, 'grid_map/depth_filter_tolerance': .15,
            'grid_map/depth_filter_maxdist': 4.0, 'grid_map/depth_filter_mindist': .3,
            'grid_map/depth_filter_margin': 2, 'grid_map/k_depth_scaling_factor': 1000.0,
            'grid_map/skip_pixel': 2,
            'grid_map/p_hit': .65, 'grid_map/p_miss': .35, 'grid_map/p_min': .12, 'grid_map/p_max': .90,
            'grid_map/p_occ': .80, 'grid_map/min_ray_length': .1, 'grid_map/max_ray_length': 4.0,
            'grid_map/virtual_ceil_height': CEIL, 'grid_map/visualization_truncate_height': 1.8,
            'grid_map/show_occ_time': False, 'grid_map/pose_type': 1, 'grid_map/frame_id': 'world',
            'grid_map/odom_depth_timeout': 2.5,   # depth arrives ~1.7 Hz
            'manager/max_vel': MAX_VEL, 'manager/max_acc': MAX_ACC, 'manager/max_jerk': 3.0,
            'manager/control_points_distance': .3, 'manager/feasibility_tolerance': .05,
            'manager/planning_horizon': HORIZON, 'manager/use_distinctive_trajs': True, 'manager/drone_id': 0,
            'optimization/lambda_smooth': 1.0, 'optimization/lambda_collision': .5,
            'optimization/lambda_feasibility': .1, 'optimization/lambda_fitness': 1.0,
            'optimization/dist0': DIST0, 'optimization/swarm_clearance': .5,
            'optimization/max_vel': MAX_VEL, 'optimization/max_acc': MAX_ACC,
            'bspline/limit_vel': MAX_VEL, 'bspline/limit_acc': MAX_ACC, 'bspline/limit_ratio': 1.1,
            'prediction/obj_num': 0, 'prediction/lambda': 1.0, 'prediction/predict_rate': 1.0,
        }])
    traj_server = Node(package='ego_planner', executable='traj_server', name='traj_server', output='screen',
                       parameters=[{'traj_server/time_forward': 1.0}])
    bridge = ExecuteProcess(cmd=['python3', os.path.join(HERE, 'ego_bridge.py')], output='screen')
    return LaunchDescription([planner, traj_server, bridge])

#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    slam_config_dir = os.path.join(
        get_package_share_directory('agv_proto'),
        'config',
        'mapper_params_online_async.yaml'
    )
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('agv_proto'),
        'config',
        'slam_rviz_config.rviz'
    )

    # Launch configuration variables
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='laser')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        # RPLidar A1M8 node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': 115200,
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'auto_reconnect': True
            }],
            output='screen'
        ),

        # Static transform: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),

        # Static transform: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'slam_toolbox': {
                    'solver_plugin': 'solver_plugins::CeresSolver',
                    'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                    'ceres_preconditioner': 'SCHUR_JACOBI',
                    'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
                    'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
                    'ceres_loss_function': 'None',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'base_frame': 'base_footprint',
                    'scan_subscriber_queue_size': 5,
                    'scan_buffer_size': 10,
                    'scan_buffer_maximum_scan_distance': 10.0,
                    'link_match_minimum_response_fine': 0.1,
                    'link_scan_maximum_distance': 1.5,
                    'loop_match_minimum_chain_size': 10,
                    'loop_match_maximum_variance_coarse': 3.0,
                    'loop_match_minimum_response_coarse': 0.35,
                    'loop_match_minimum_response_fine': 0.45,
                    'correlation_search_space_dimension': 0.5,
                    'correlation_search_space_resolution': 0.01,
                    'correlation_search_space_smear_deviation': 0.1,
                    'loop_search_space_dimension': 8.0,
                    'loop_search_space_resolution': 0.05,
                    'loop_search_space_smear_deviation': 0.03,
                    'distance_variance_penalty': 0.5,
                    'angle_variance_penalty': 1.0,
                    'fine_search_angle_offset': 0.00349,
                    'coarse_search_angle_offset': 0.349,
                    'coarse_angle_resolution': 0.0349,
                    'minimum_angle_penalty': 0.9,
                    'minimum_distance_penalty': 0.5,
                    'use_angle_variance_penalty': False,
                    'use_scan_matching': True,
                    'use_scan_barycenter': True,
                    'minimum_travel_distance': 0.5,
                    'minimum_travel_heading': 0.5,
                    'scan_queue_size': 5,
                    'scan_topic': '/scan',
                    'map_file_name': '/tmp/my_map',
                    'map_start_pose': [0.0, 0.0, 0.0],
                    'debug_logging': False
                }
            }],
        ),

        # Robot State Publisher for odom -> base_footprint transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])

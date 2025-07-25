#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
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

        # Static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
            output='screen'
        ),

        # Lidar node with higher frequency
        TimerAction(
            period=2.0,
            actions=[
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
                        'auto_reconnect': True,
                        'scan_frequency': 15.0  # Higher frequency for more responsive mapping
                    }],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0
                ),
            ]
        ),

        # SLAM with VERY RESPONSIVE settings
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        
                        # Basic solver
                        'solver_plugin': 'solver_plugins::CeresSolver',
                        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                        'ceres_preconditioner': 'SCHUR_JACOBI',
                        'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
                        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
                        'ceres_loss_function': 'None',
                        
                        # Frames
                        'odom_frame': 'odom',
                        'map_frame': 'map',
                        'base_frame': 'base_footprint',
                        
                        # ULTRA RESPONSIVE SETTINGS - Update map with minimal movement!
                        'minimum_travel_distance': 0.05,    # Update every 5cm movement (was 0.3)
                        'minimum_travel_heading': 0.05,     # Update every 3 degrees rotation (was 0.3)
                        'minimum_time_interval': 0.1,       # Process scans every 0.1 seconds (was 0.5)
                        'transform_publish_period': 0.01,   # 100Hz transforms (was 0.02)
                        'map_update_interval': 0.5,         # Update map every 0.5 seconds (was 2.0)
                        
                        # Scan processing - more aggressive
                        'scan_subscriber_queue_size': 10,   # Larger queue for faster processing
                        'scan_buffer_size': 20,
                        'scan_buffer_maximum_scan_distance': 12.0,
                        'max_laser_range': 12.0,
                        'throttle_scans': 1,                # Process EVERY scan
                        'scan_topic': 'scan',
                        
                        # Map settings
                        'resolution': 0.03,                 # Higher resolution (3cm vs 5cm)
                        'use_scan_matching': True,
                        'use_scan_barycenter': True,
                        
                        # Loop closure - more sensitive
                        'link_match_minimum_response_fine': 0.05,  # More sensitive (was 0.1)
                        'link_scan_maximum_distance': 2.0,         # Larger matching distance
                        'loop_match_minimum_chain_size': 5,        # Smaller chain for faster loops
                        'loop_match_maximum_variance_coarse': 5.0,
                        'loop_match_minimum_response_coarse': 0.2, # More sensitive
                        'loop_match_minimum_response_fine': 0.3,   # More sensitive
                        
                        # Correlation - faster search
                        'correlation_search_space_dimension': 0.3,
                        'correlation_search_space_resolution': 0.005,
                        'correlation_search_space_smear_deviation': 0.05,
                        
                        # Performance
                        'debug_logging': False,
                        'tf_buffer_duration': 10.0,         # Shorter buffer
                        'transform_timeout': 0.1,           # Faster timeout
                        'stack_size_to_use': 40000000,
                        
                        # Enable interactive mode for better real-time performance
                        'enable_interactive_mode': True,
                        
                        # Additional responsive settings
                        'map_file_name': '/tmp/live_responsive_map',
                        'map_start_pose': [0.0, 0.0, 0.0],
                        'first_map_only': False,
                        'map_start_at_dock': True
                    }],
                    remappings=[
                        ('/scan', '/scan'),
                        ('/map', '/map'),
                        ('/map_metadata', '/map_metadata')
                    ]
                ),
            ]
        ),
    ])

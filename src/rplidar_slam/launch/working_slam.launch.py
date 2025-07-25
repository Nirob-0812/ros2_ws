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

        # Static transforms first
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

        # Wait a bit then start lidar
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
                        'scan_frequency': 10.0
                    }],
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0
                ),
            ]
        ),

        # Wait longer then start SLAM
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
                        # Basic SLAM parameters
                        'solver_plugin': 'solver_plugins::CeresSolver',
                        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                        'ceres_preconditioner': 'SCHUR_JACOBI',
                        'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
                        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
                        'ceres_loss_function': 'None',
                        
                        # Frame settings
                        'odom_frame': 'odom',
                        'map_frame': 'map',
                        'base_frame': 'base_footprint',
                        
                        # Scan processing
                        'scan_subscriber_queue_size': 5,
                        'scan_buffer_size': 10,
                        'scan_buffer_maximum_scan_distance': 12.0,
                        'max_laser_range': 12.0,
                        'minimum_travel_distance': 0.3,
                        'minimum_travel_heading': 0.3,
                        'scan_topic': 'scan',
                        'use_scan_matching': True,
                        'use_scan_barycenter': True,
                        'minimum_time_interval': 0.5,
                        'transform_publish_period': 0.02,
                        'map_update_interval': 2.0,
                        'resolution': 0.05,
                        
                        # Loop closure
                        'link_match_minimum_response_fine': 0.1,
                        'link_scan_maximum_distance': 1.5,
                        'loop_match_minimum_chain_size': 10,
                        'loop_match_maximum_variance_coarse': 3.0,
                        'loop_match_minimum_response_coarse': 0.35,
                        'loop_match_minimum_response_fine': 0.45,
                        
                        # Other settings
                        'debug_logging': False,
                        'throttle_scans': 1,
                        'tf_buffer_duration': 30.0,
                        'transform_timeout': 0.2
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

#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # LIDAR parameters
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # SLAM parameters embedded directly
    slam_params = {
        'use_sim_time': False,
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'map_frame': 'map',
        'scan_topic': '/scan',
        'mode': 'mapping',
        'debug_logging': True,
        'throttle_scans': 1,
        'transform_publish_period': 0.02,
        'map_update_interval': 5.0,
        'resolution': 0.05,
        'max_laser_range': 12.0,
        'minimum_time_interval': 0.5,
        'transform_timeout': 0.2,
        'tf_buffer_duration': 30.0,
        'stack_size_to_use': 40000000,
        'enable_interactive_mode': True
    }

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='laser'),

        # 1. LIDAR Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port, 
                'serial_baudrate': serial_baudrate, 
                'frame_id': frame_id,
                'inverted': inverted, 
                'angle_compensate': angle_compensate, 
                'scan_mode': scan_mode
            }],
            output='screen'),

        # 2. Static Transform Publisher (odom -> base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0', 
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'odom', '--child-frame-id', 'base_link'],
            output='screen'),

        # 3. Static Transform Publisher (base_link -> laser)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0.1', 
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'base_link', '--child-frame-id', 'laser'],
            output='screen'),

        # 4. SLAM Toolbox Node with embedded parameters
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen'),

        # 5. RViz2 Node (without custom config to avoid issues)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])

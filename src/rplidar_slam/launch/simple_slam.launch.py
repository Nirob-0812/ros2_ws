#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
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

        # Lidar node
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

        # SLAM with simpler, more reliable settings
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
                         '--ros-args',
                         '-p', 'use_sim_time:=false',
                         '-p', 'odom_frame:=odom',
                         '-p', 'map_frame:=map', 
                         '-p', 'base_frame:=base_footprint',
                         '-p', 'scan_topic:=scan',
                         '-p', 'minimum_travel_distance:=0.2',
                         '-p', 'minimum_travel_heading:=0.2',
                         '-p', 'minimum_time_interval:=0.5',
                         '-p', 'transform_publish_period:=0.02',
                         '-p', 'map_update_interval:=1.0',
                         '-p', 'resolution:=0.05',
                         '-p', 'max_laser_range:=12.0',
                         '-p', 'use_scan_matching:=true',
                         '-p', 'debug_logging:=false'],
                    output='screen',
                    shell=False
                ),
            ]
        ),
    ])

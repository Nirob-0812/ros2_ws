#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
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

    # SLAM parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get the package directory
    pkg_dir = get_package_share_directory('sllidar_ros2')

    # RViz config for SLAM
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'slam_mapping.rviz')
    if not os.path.exists(rviz_config_dir):
        rviz_config_dir = os.path.join(os.path.dirname(__file__), '..', 'rviz', 'slam_mapping.rviz')
        rviz_config_dir = os.path.abspath(rviz_config_dir)

    # Embedded SLAM parameters (no external file needed)
    slam_params = {
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',
        'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
        'ceres_loss_function': 'None',
        
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_link',
        'scan_topic': '/scan',
        'mode': 'mapping',
        
        'map_update_interval': 2.0,
        'resolution': 0.05,
        'max_laser_range': 12.0,
        'minimum_time_interval': 0.5,
        'transform_publish_period': 0.02,
        'transform_timeout': 0.2,
        'tf_buffer_duration': 30.0,
        'stack_size_to_use': 40000000,
        'enable_interactive_mode': True,
        
        'use_scan_matching': True,
        'use_scan_barycenter': True,
        'minimum_travel_distance': 0.2,
        'minimum_travel_heading': 0.2,
        'scan_buffer_size': 10,
        'scan_buffer_maximum_scan_distance': 10.0,
        'link_match_minimum_response_fine': 0.1,
        'link_scan_maximum_distance': 1.5,
        'loop_search_maximum_distance': 3.0,
        'do_loop_closing': True,
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
        'use_response_expansion': True,
        
        'throttle_scans': 1,
        'debug_logging': False,
        'use_sim_time': use_sim_time
    }

    return LaunchDescription([
        # LIDAR parameters
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        # SLAM parameters
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # LIDAR Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate, 
                         'scan_mode': scan_mode}],
            output='screen'),

        # Static Transform Publisher (base_link to laser frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0.1', 
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'base_link', '--child-frame-id', 'laser'],
            output='screen'),

        # Robot State Publisher (publishes robot description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': '''<?xml version="1.0"?>
                    <robot name="lidar_robot">
                        <link name="base_link">
                            <visual>
                                <geometry>
                                    <cylinder length="0.1" radius="0.05"/>
                                </geometry>
                                <material name="blue">
                                    <color rgba="0 0 1 1"/>
                                </material>
                            </visual>
                        </link>
                        <link name="laser">
                            <visual>
                                <geometry>
                                    <cylinder length="0.02" radius="0.02"/>
                                </geometry>
                                <material name="red">
                                    <color rgba="1 0 0 1"/>
                                </material>
                            </visual>
                        </link>
                        <joint name="base_to_laser" type="fixed">
                            <parent link="base_link"/>
                            <child link="laser"/>
                            <origin xyz="0 0 0.1" rpy="0 0 0"/>
                        </joint>
                    </robot>
                ''',
                'use_sim_time': use_sim_time
            }],
            output='screen'),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen'),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])

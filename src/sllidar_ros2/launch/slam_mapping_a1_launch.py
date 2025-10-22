#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    # SLAM parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Get the package directory
    pkg_dir = get_package_share_directory('sllidar_ros2')
    
    # Set default SLAM params file - try multiple locations
    default_slam_params_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    
    # Fallback to source if install doesn't exist
    if not os.path.exists(default_slam_params_file):
        default_slam_params_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'slam_params.yaml')
        default_slam_params_file = os.path.abspath(default_slam_params_file)

    # RViz config for SLAM
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'slam_mapping.rviz')
    if not os.path.exists(rviz_config_dir):
        rviz_config_dir = os.path.join(os.path.dirname(__file__), '..', 'rviz', 'slam_mapping.rviz')
        rviz_config_dir = os.path.abspath(rviz_config_dir)

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

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params_file,
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),

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

        # Static Transform Publisher (odom to base_link frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0', 
                      '--roll', '0', '--pitch', '0', '--yaw', '0',
                      '--frame-id', 'odom', '--child-frame-id', 'base_link'],
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

        # SLAM Toolbox Node - MINIMAL WORKING VERSION
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'base_frame': 'base_link', 
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping'
            }],
            output='screen'),

        # RViz2 Node - Simple version without config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])

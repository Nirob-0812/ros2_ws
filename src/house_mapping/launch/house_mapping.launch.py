from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{'serial_port': '/dev/ttyUSB0'}],
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/opt/ros/jazzy/share/slam_toolbox/launch/slam_toolbox.rviz'],
            output='screen'
        ),
    ])

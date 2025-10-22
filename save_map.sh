#!/bin/bash

# Simple map saver script
echo "Saving map..."

# Source ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Save the map
ros2 run nav2_map_server map_saver_cli -f my_house_map

echo "Map saved as my_house_map.pgm and my_house_map.yaml"
echo "Files saved in: $(pwd)"

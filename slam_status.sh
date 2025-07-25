#!/bin/bash

# Quick SLAM Status Check
# ======================

echo "🔍 SLAM Status Diagnostic"
echo "========================"
echo ""

source ~/ros2_ws/install/setup.bash

echo "1. 🤖 ROS Nodes:"
ros2 node list | grep -E 'slam|sllidar' || echo "No SLAM/lidar nodes found"

echo ""
echo "2. 📡 Topics:"
ros2 topic list | grep -E 'scan|map|tf'

echo ""
echo "3. 🗺️ Map Topic Details:"
ros2 topic info /map

echo ""
echo "4. 📊 Map Data Test:"
echo "Trying to get map data (5 second timeout)..."
timeout 5s ros2 topic echo /map --once | head -20

echo ""
echo "5. 🔄 Services:"
ros2 service list | grep slam_toolbox

echo ""
echo "6. 💾 Try Manual Map Save:"
echo "ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '/tmp/test_map'}}\""

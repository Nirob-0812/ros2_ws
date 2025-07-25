#!/bin/bash

# Fixed SLAM Launch Script
# ========================
# This script properly sets up all transforms for SLAM mapping

echo "Fixed SLAM System Launch"
echo "========================"
echo ""

echo "1. Killing any existing processes..."
pkill -f "slam_toolbox"
pkill -f "sllidar"
pkill -f "rviz"
sleep 2

echo "2. Sourcing ROS environment..."
source ~/ros2_ws/install/setup.bash

echo "3. Building workspace to include new launch file..."
cd ~/ros2_ws
colcon build --packages-select agv_proto
source install/setup.bash

echo "4. Checking lidar connection..."
if [ -e /dev/ttyUSB0 ]; then
    echo "   ✓ Lidar found at /dev/ttyUSB0"
    sudo chmod 666 /dev/ttyUSB0
else
    echo "   ✗ No lidar found at /dev/ttyUSB0"
    echo "   Available USB devices:"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "   No USB devices found"
fi

echo ""
echo "5. Launching Fixed SLAM system..."
echo "   This will start:"
echo "   - RPLidar A1M8 driver"
echo "   - SLAM Toolbox with proper transforms"
echo "   - RViz for visualization"
echo "   - All necessary transform publishers"
echo ""
echo "   The map should now be visible in RViz with Fixed Frame set to 'map'"
echo ""

# Launch the fixed SLAM system
ros2 launch agv_proto fixed_slam.launch.py

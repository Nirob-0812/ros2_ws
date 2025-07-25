#!/bin/bash

# Ultra-Responsive Live SLAM - See Map Updates in Real-Time!
# ==========================================================

echo "🔥 Ultra-Responsive Live SLAM"
echo "============================="
echo ""
echo "This version updates the map with EVERY small movement!"
echo "You'll see live map building as you move the lidar!"
echo ""

echo "1. Stopping existing processes..."
pkill -f "slam_toolbox" 2>/dev/null
pkill -f "sllidar" 2>/dev/null
pkill -f "rviz" 2>/dev/null
sleep 2

echo "2. Building workspace with live SLAM..."
cd ~/ros2_ws
colcon build --packages-select agv_proto --symlink-install
source install/setup.bash

echo "3. Checking lidar..."
if [ -e /dev/ttyUSB0 ]; then
    echo "   ✅ Lidar ready at /dev/ttyUSB0"
    sudo chmod 666 /dev/ttyUSB0
else
    echo "   ❌ Lidar not found!"
    exit 1
fi

echo ""
echo "🚀 Starting Ultra-Responsive SLAM..."
echo ""
echo "📋 What you'll see:"
echo "   • Map updates every 5cm of movement"
echo "   • Higher resolution (3cm pixels)"
echo "   • Faster scan processing"
echo "   • Real-time map building"
echo ""
echo "🎮 To test:"
echo "   1. Move the lidar slowly around"
echo "   2. You should see the map grow immediately"
echo "   3. Try rotating the lidar - map should update instantly"
echo ""
echo "🖥️  Open RViz in another terminal: ./run_rviz_fixed.sh"
echo ""

# Launch ultra-responsive SLAM
ros2 launch agv_proto live_slam.launch.py

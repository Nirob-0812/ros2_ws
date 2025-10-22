#!/bin/bash

# Quick SLAM starter script
echo "🚀 Starting SLAM Mapping for RPLIDAR A1..."
echo ""

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "✅ ROS2 environment sourced"

# Kill any existing nodes
echo "🔄 Cleaning up any existing nodes..."
pkill -f "sllidar_node|slam_toolbox|rviz2|static_transform_publisher|robot_state_publisher" 2>/dev/null
sleep 2

echo "🎯 Starting SLAM system..."

# Start each component individually for better debugging
echo "1️⃣ Starting LIDAR node..."
ros2 run sllidar_ros2 sllidar_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=true \
  -p scan_mode:=Sensitivity &
LIDAR_PID=$!
sleep 5

# Wait for LIDAR to be fully ready
echo "   Waiting for LIDAR to initialize..."
while ! ros2 topic list | grep -q "/scan"; do
    sleep 1
done
echo "   ✅ LIDAR is ready!"

echo "2️⃣ Starting transform publishers..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.1 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id laser &
TF_PID=$!
sleep 1

echo "3️⃣ Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='<?xml version="1.0"?><robot name="lidar_robot"><link name="base_link"><visual><geometry><cylinder length="0.1" radius="0.05"/></geometry></visual></link><link name="laser"><visual><geometry><cylinder length="0.02" radius="0.02"/></geometry></visual></link><joint name="base_to_laser" type="fixed"><parent link="base_link"/><child link="laser"/><origin xyz="0 0 0.1" rpy="0 0 0"/></joint></robot>' &
RSP_PID=$!
sleep 2

echo "4️⃣ Starting SLAM toolbox..."
ros2 run slam_toolbox sync_slam_toolbox_node --ros-args \
  -p odom_frame:=odom \
  -p map_frame:=map \
  -p base_frame:=base_link \
  -p scan_topic:=/scan \
  -p mode:=mapping \
  -p resolution:=0.05 \
  -p max_laser_range:=12.0 \
  -p minimum_travel_distance:=0.05 \
  -p minimum_travel_heading:=0.05 \
  -p map_update_interval:=1.0 \
  -p transform_publish_period:=0.02 \
  -p use_scan_matching:=true \
  -p do_loop_closing:=true \
  -p enable_interactive_mode:=true &
SLAM_PID=$!
sleep 5

# Wait for SLAM to start processing
echo "   Waiting for SLAM to start..."
while ! ros2 topic list | grep -q "/map"; do
    sleep 1
done
echo "   ✅ SLAM toolbox is ready!"

echo "5️⃣ Starting RViz..."
ros2 run rviz2 rviz2 &
RVIZ_PID=$!

echo ""
echo "🎉 All nodes started!"
echo ""
echo "📋 Process IDs:"
echo "   LIDAR: $LIDAR_PID"
echo "   Transform: $TF_PID" 
echo "   Robot State: $RSP_PID"
echo "   SLAM: $SLAM_PID"
echo "   RViz: $RVIZ_PID"
echo ""
echo "⏳ Wait 5 seconds for everything to stabilize..."
sleep 5

echo "🔍 Checking system status..."

# Check if scan is being published
if timeout 3 ros2 topic hz /scan --window 3 --count 2 >/dev/null 2>&1; then
    echo "✅ Laser scan data is flowing"
    
    # Check if there's actual scan data
    echo "   Checking scan data quality..."
    if timeout 3 ros2 topic echo /scan --once --field ranges | head -5 >/dev/null 2>&1; then
        echo "   ✅ Scan data contains valid ranges"
    fi
else
    echo "❌ No laser scan data detected"
fi

# Give SLAM a moment to see some scans
sleep 3

# Check if map frame exists
if timeout 3 ros2 run tf2_ros tf2_echo map odom &>/dev/null; then
    echo "✅ Map frame is available"
    echo ""
    echo "🎯 SUCCESS! You can now:"
    echo "   1. In RViz, change Fixed Frame to 'map'"
    echo "   2. Add Map display (topic: /map)"
    echo "   3. Start moving around to build the map!"
else
    echo "⚠️  Map frame not ready yet"
    echo "   This is normal - SLAM needs some movement to start"
    echo "   1. In RViz, set Fixed Frame to 'laser' for now"
    echo "   2. Move the LIDAR around slightly"
    echo "   3. After movement, change Fixed Frame to 'map'"
fi

echo ""
echo "🛑 To stop all nodes: pkill -f 'sllidar_node|slam_toolbox|rviz2|static_transform_publisher|robot_state_publisher'"

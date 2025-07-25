#!/bin/bash

# Cleanup Script for AGV Project
# ==============================

echo "🧹 Cleaning up unnecessary files in AGV project..."

# Create backup directory
echo "📁 Creating backup directory..."
mkdir -p ~/ros2_ws_backup

# Remove unnecessary packages (keeping only what's needed for SLAM)
echo "🗑️ Cleaning up navigation2 build files..."
rm -rf ~/ros2_ws/build/dwb_msgs
rm -rf ~/ros2_ws/build/nav_2d_msgs  
rm -rf ~/ros2_ws/build/nav2_common
rm -rf ~/ros2_ws/build/nav2_gazebo_spawner
rm -rf ~/ros2_ws/build/nav2_msgs
rm -rf ~/ros2_ws/build/nav2_util
rm -rf ~/ros2_ws/build/nav2_voxel_grid

echo "🗑️ Cleaning up install files..."
rm -rf ~/ros2_ws/install/dwb_msgs
rm -rf ~/ros2_ws/install/nav_2d_msgs
rm -rf ~/ros2_ws/install/nav2_common
rm -rf ~/ros2_ws/install/nav2_gazebo_spawner
rm -rf ~/ros2_ws/install/nav2_msgs
rm -rf ~/ros2_ws/install/nav2_voxel_grid

echo "🗑️ Cleaning up old log files..."
find ~/ros2_ws/log -name "*.log" -mtime +7 -delete

echo "📦 Moving navigation2 source to backup (not needed for SLAM)..."
mv ~/ros2_ws/src/navigation2 ~/ros2_ws_backup/ 2>/dev/null || echo "Navigation2 already moved or not found"

echo "🧹 Cleaning up temporary files..."
find ~/ros2_ws -name "*.pyc" -delete
find ~/ros2_ws -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null

echo "✅ Cleanup completed!"
echo ""
echo "📁 Backup created at: ~/ros2_ws_backup"
echo "🎯 Your workspace now contains only:"
echo "   - agv_proto (your main package with SLAM)"
echo "   - sllidar_ros2 (lidar driver)"  
echo "   - ros_odrive_msg (motor messages)"
echo ""
echo "🚀 Ready to run SLAM! Use: ./run_slam.sh"

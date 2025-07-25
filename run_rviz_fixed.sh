#!/bin/bash

# Launch RViz with Graphics Workaround
# ====================================

echo "🎨 Starting RViz with Graphics Workaround"
echo "========================================="
echo ""

echo "1. Sourcing environment..."
source ~/ros2_ws/install/setup.bash

echo "2. Setting graphics environment variables..."
export LIBGL_ALWAYS_INDIRECT=0
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

echo "3. Launching RViz..."
echo "   If you see graphics errors, the SLAM is still working!"
echo "   Check terminal output for mapping progress."
echo ""

# Launch RViz with the SLAM config
rviz2 -d src/agv_proto/config/slam_rviz_config.rviz

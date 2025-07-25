# RPLidar A1M8 SLAM Setup - Quick Start Guide

## What We've Created

I've set up a complete SLAM (Simultaneous Localization and Mapping) system for your RPLidar A1M8. Here's what's been configured:

### New Files Created:
1. **`~/ros2_ws/src/agv_proto/launch/lidar_slam.launch.py`** - Complete SLAM launch file
2. **`~/ros2_ws/src/agv_proto/config/mapper_params_online_async.yaml`** - SLAM Toolbox configuration
3. **`~/ros2_ws/src/agv_proto/config/slam_rviz_config.rviz`** - Custom RViz configuration for SLAM
4. **`~/ros2_ws/run_slam.sh`** - Easy script to start SLAM
5. **`~/ros2_ws/save_map.sh`** - Script to save your map

## How to Use

### 1. Start SLAM Mapping
Connect your RPLidar A1M8 to USB port and run:
```bash
cd ~/ros2_ws
./run_slam.sh
```

Or manually:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch agv_proto lidar_slam.launch.py
```

### 2. What You'll See
- **RViz window** with live lidar data and map building in real-time
- **Red dots**: Lidar points
- **Gray/Black areas**: Your map (black = obstacles, gray = free space)
- **Coordinate frames**: Showing robot position

### 3. Create Your Map
- **Move around your house** slowly with the lidar
- Watch the map build in real-time in RViz
- Make sure to cover all areas you want mapped

### 4. Save Your Map
When you're happy with the map:
```bash
# In a new terminal
cd ~/ros2_ws
./save_map.sh my_house_map
```

### 5. Troubleshooting

**If lidar not detected:**
```bash
# Check USB port
ls /dev/ttyUSB*
# If it's not ttyUSB0, launch with:
ros2 launch agv_proto lidar_slam.launch.py serial_port:=/dev/ttyUSB1
```

**If no map appears:**
- Make sure you're moving the lidar around
- Check that lidar is spinning and red laser is visible
- Verify the transform frames in RViz (TF display)

## What's Different from Before

**Before:** Only lidar points were visible
**Now:** 
- Real-time map building with SLAM Toolbox
- Proper coordinate frame transformations
- Optimized RViz display for mapping
- Easy save functionality

The system now includes:
- **Lidar driver** (sllidar_ros2)
- **SLAM algorithm** (slam_toolbox)
- **Map visualization** (RViz)
- **Transform publishers** (tf2)

Your maps will be saved in `~/ros2_ws/src/agv_proto/maps/` as both .yaml and .pgm files.

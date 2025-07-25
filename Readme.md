# RPLidar A1M8 SLAM Setup Guide - Complete Installation & Usage

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [ROS2 Foxy Installation](#ros2-foxy-installation)
3. [Workspace Setup](#workspace-setup)
4. [Package Installation](#package-installation)
5. [Hardware Setup](#hardware-setup)
6. [Running SLAM](#running-slam)
7. [Viewing Saved Maps](#viewing-saved-maps)
8. [Troubleshooting](#troubleshooting)

---

## Prerequisites

- Ubuntu 20.04 LTS (or Windows with WSL2 Ubuntu 20.04)
- RPLidar A1M8 sensor
- USB cable for lidar connection

---

## ROS2 Foxy Installation

### Step 1: Setup Locale
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Setup Sources
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS2 Foxy
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop python3-colcon-common-extensions
```

### Step 4: Environment Setup
```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Workspace Setup

### Step 1: Create ROS2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2: Clone Required Packages
```bash
# Clone sllidar_ros2 (RPLidar driver)
git clone https://github.com/Slamtec/sllidar_ros2.git

# Clone your RPLidar SLAM package (if not already present)
# git clone <your-rplidar-slam-repo-url>
```

---

## Package Installation

### Step 1: Install Dependencies
```bash
cd ~/ros2_ws
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 2: Install Additional Packages
```bash
sudo apt install ros-foxy-slam-toolbox ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-turtlebot3*
```

### Step 3: Build Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Hardware Setup

### Step 1: Connect RPLidar A1M8
1. Connect your RPLidar A1M8 to USB port
2. Check if detected:
```bash
ls /dev/ttyUSB*
```
You should see `/dev/ttyUSB0` (or similar)

### Step 2: Set USB Permissions
```bash
sudo chmod 666 /dev/ttyUSB0
```

**Note:** You may need to run this command every time you reconnect the lidar.

---

## Running SLAM

### Method 1: Using Launch File (Recommended)
```bash
cd ~/ros2_ws
source install/setup.bash
sudo chmod 666 /dev/ttyUSB0
ros2 launch rplidar_slam mapping.launch.py
```

### Method 2: Using Script
```bash
cd ~/ros2_ws
chmod +x run_fixed_slam.sh
./run_fixed_slam.sh
```

### What You'll See:
- **RViz window** opens with live lidar visualization
- **Red/colored dots**: Live lidar scan points
- **Black/gray areas**: Map being built in real-time
  - **Black**: Obstacles (walls, furniture)
  - **Gray**: Free space (walkable areas)
  - **White**: Unknown/unexplored areas

### Creating Your Map:
1. **Move slowly** around your house with the lidar
2. **Cover all areas** you want mapped
3. **Watch the map build** in real-time in RViz
4. **Ensure good coverage** of all rooms and corridors

---

## Saving Your Map

### Save Current Map:
```bash
# In a new terminal while SLAM is running
cd ~/ros2_ws
chmod +x save_map.sh
./save_map.sh
```

The map will be saved as:
- `~/ros2_ws/maps/house_map.pgm` (image file)
- `~/ros2_ws/maps/house_map.yaml` (metadata file)

---

## Viewing Saved Maps

### View Previously Saved Map:
```bash
cd ~/ros2_ws
chmod +x view_map.sh
./view_map.sh house_map
```

### Manual Method:
```bash
# Start map server
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/ros2_ws/maps/house_map.yaml &

# Start RViz
rviz2 &

# In RViz:
# 1. Set Fixed Frame to "map"
# 2. Add "Map" display
# 3. Set Map topic to "/map"
```

---

## Troubleshooting

### Problem: "No such file or directory" for lidar
**Solution:**
```bash
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0  # Use the correct USB device
```

### Problem: "Transport endpoint is not connected"
**Solution:**
```bash
# Kill existing processes and restart
pkill -f "sllidar"
sudo chmod 666 /dev/ttyUSB0
# Then restart SLAM
```

### Problem: RViz shows only grid, no map
**Solution:**
1. In RViz, click "Add" button
2. Select "Map" from the list
3. Set Topic to "/map"
4. Set Fixed Frame to "map" or "odom"

### Problem: Map is not updating/stuck
**Solution:**
```bash
# Check if topics are being published
ros2 topic list | grep -E "(scan|map)"
ros2 topic hz /scan
ros2 topic hz /map

# Restart SLAM if needed
./cleanup.sh
./run_fixed_slam.sh
```

### Problem: Permission denied on USB
**Solution:**
```bash
# Add user to dialout group (one-time setup)
sudo usermod -a -G dialout $USER
# Then log out and log back in

# Or set permissions each time
sudo chmod 666 /dev/ttyUSB0
```

---

## Available Scripts

- **`run_fixed_slam.sh`** - Start SLAM mapping
- **`save_map.sh`** - Save current map
- **`view_map.sh`** - View saved maps
- **`slam_status.sh`** - Check SLAM system status
- **`cleanup.sh`** - Clean up running processes

---

## Quick Start Summary

1. **Install ROS2 Foxy** (follow installation section)
2. **Setup workspace** and build packages
3. **Connect lidar** and set permissions: `sudo chmod 666 /dev/ttyUSB0`
4. **Start mapping**: `ros2 launch rplidar_slam mapping.launch.py`
5. **Move around** to build map
6. **Save map**: `./save_map.sh`
7. **View saved map**: `./view_map.sh house_map`

---

## Success Indicators

✅ **Lidar connected**: You see `/dev/ttyUSB0` when running `ls /dev/ttyUSB*`  
✅ **SLAM running**: RViz opens and shows lidar points  
✅ **Mapping working**: You see black/gray areas building as you move  
✅ **Map saved**: Files created in `~/ros2_ws/maps/`  
✅ **Map viewable**: Saved map displays correctly in RViz  

**Congratulations! Your RPLidar A1M8 SLAM system is working! 🎉**

**If lidar not detected:**
```bash
# Check USB port
ls /dev/ttyUSB*
# If it's not ttyUSB0, launch with:
ros2 launch rplidar_slam mapping.launch.py serial_port:=/dev/ttyUSB1
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

Your maps will be saved in `~/ros2_ws/maps/` as both .yaml and .pgm files.

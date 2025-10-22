# SLAM Mapping Guide for RPLIDAR A1

This guide will help you create a live map of your house using SLAM (Simultaneous Localization and Mapping) with your RPLIDAR A1.

## Prerequisites

1. **RPLIDAR A1** connected to `/dev/ttyUSB0` (or adjust the serial port in the launch file)
2. **ROS2 Jazzy** installed
3. **SLAM Toolbox** installed (should be installed automatically)
4. **Workspace built** with `colcon build`

## Quick Start

### 1. Launch SLAM Mapping

```bash
# Terminal 1: Start SLAM mapping
cd ~/ros2_ws
source install/setup.bash
ros2 launch sllidar_ros2 slam_mapping_a1_launch.py
```

This will start:
- RPLIDAR A1 node (publishing laser scans)
- SLAM Toolbox (creating the map)
- RViz2 (for visualization)
- Robot State Publisher (for coordinate frames)

### 2. What You'll See in RViz

- **White/colored dots**: Real-time laser scan points
- **Black/gray areas**: Walls and obstacles (mapped areas)
- **Gray areas**: Free space (areas you can move through)
- **Unknown areas**: Areas not yet explored

### 3. Mapping Your House

**Important Tips:**
- Move **slowly and steadily** - don't rush!
- **Overlap your paths** - revisit areas to improve map quality
- **Go around corners carefully** - ensure the LIDAR can see both sides
- **Close loops** - return to previously mapped areas to help SLAM correct drift
- **Avoid quick rotations** - smooth movements work best

**Recommended Mapping Pattern:**
1. Start in a central room
2. Map one room completely before moving to the next
3. Always return through mapped areas to reach new rooms
4. Walk close to walls and furniture to get good detail
5. End where you started to "close the loop"

### 4. Monitor Mapping Quality

Watch RViz for:
- **Consistent walls**: Should be straight and not wavy
- **Proper loop closure**: When you return to a mapped area, it should align well
- **No duplicate walls**: If you see double walls, move more slowly

### 5. Save Your Map

When you're satisfied with your map:

```bash
# Terminal 2: Save the map
cd ~/ros2_ws
source install/setup.bash
./src/sllidar_ros2/scripts/save_map.sh my_house_map
```

This creates:
- `my_house_map.yaml` - Map configuration file
- `my_house_map.pgm` - Map image file

Both files are saved in `~/ros2_ws/maps/`

## Troubleshooting

### LIDAR Not Detected
```bash
# Check if LIDAR is connected
ls -la /dev/ttyUSB*

# If using different port, launch with:
ros2 launch sllidar_ros2 slam_mapping_a1_launch.py serial_port:=/dev/ttyUSB1
```

### Poor Map Quality
- Move more slowly
- Ensure LIDAR is level and stable
- Check for reflective surfaces (mirrors, glass) that might confuse the LIDAR
- Improve lighting (some LIDARs perform better in good lighting)

### RViz Not Showing Map
- Check that the `/map` topic is being published: `ros2 topic echo /map`
- Verify SLAM toolbox is running: `ros2 node list | grep slam`

### Map Drift
- Ensure you're creating loop closures (returning to previously mapped areas)
- Move more slowly
- The map will automatically correct when you close loops

## Advanced Usage

### Custom SLAM Parameters

Edit `src/sllidar_ros2/config/slam_params.yaml` to adjust:
- `resolution`: Map resolution (0.05 = 5cm per pixel)
- `max_laser_range`: Maximum range to use from LIDAR
- `minimum_travel_distance`: How far to move before updating map

### Different Serial Port

```bash
ros2 launch sllidar_ros2 slam_mapping_a1_launch.py serial_port:=/dev/ttyUSB1
```

### Save Map with Different Name

```bash
./src/sllidar_ros2/scripts/save_map.sh bedroom_map
```

## File Locations

- **Launch file**: `src/sllidar_ros2/launch/slam_mapping_a1_launch.py`
- **SLAM config**: `src/sllidar_ros2/config/slam_params.yaml`
- **RViz config**: `src/sllidar_ros2/rviz/slam_mapping.rviz`
- **Save script**: `src/sllidar_ros2/scripts/save_map.sh`
- **Saved maps**: `~/ros2_ws/maps/`

## Tips for Best Results

1. **Start with a small area** and gradually expand
2. **Keep the LIDAR height consistent** throughout mapping
3. **Map systematically** - don't jump randomly between rooms
4. **Take your time** - good maps require patience
5. **Practice in one room first** to get familiar with the process

Happy mapping! 🗺️

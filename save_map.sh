#!/bin/bash

# Save Map Script - Enhanced Version
# ==================================

echo "🗺️  Map Saving Script"
echo "==================="
echo ""

echo "Setting up ROS 2 environment..."
source ~/ros2_ws/install/setup.bash

# Create maps directory
MAP_DIR="$HOME/ros2_ws/maps"
echo "Creating maps directory: $MAP_DIR"
mkdir -p "$MAP_DIR"

# Check if map name is provided
if [ -z "$1" ]; then
    MAP_NAME="house_map_$(date +%Y%m%d_%H%M%S)"
    echo "No map name provided, using: $MAP_NAME"
else
    MAP_NAME=$1
    echo "Using provided map name: $MAP_NAME"
fi

echo ""
echo "📡 Checking if map topic is available..."
ros2 topic list | grep -q "/map"
if [ $? -eq 0 ]; then
    echo "   ✅ Map topic found!"
    
    # Check if map is actually being published
    echo "🔍 Checking if map data is being published..."
    PUBLISHER_COUNT=$(ros2 topic info /map | grep "Publisher count:" | awk '{print $3}')
    
    if [ "$PUBLISHER_COUNT" -eq 0 ]; then
        echo "   ⚠️  Map topic exists but no publishers found!"
        echo "   🔧 Trying to trigger map publication..."
        
        # Send a service call to SLAM to save map
        echo "   📞 Calling SLAM save_map service..."
        ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$MAP_NAME'}}" &
        SERVICE_PID=$!
        sleep 2
        
        # Try alternative method - serialize map
        echo "   💾 Trying serialize_map service..."
        ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: {data: '/tmp/${MAP_NAME}'}}" &
        sleep 2
        
        # Check again if publisher appeared
        PUBLISHER_COUNT=$(ros2 topic info /map | grep "Publisher count:" | awk '{print $3}')
    fi
    
    if [ "$PUBLISHER_COUNT" -gt 0 ]; then
        echo "   ✅ Map publisher active ($PUBLISHER_COUNT publisher(s))"
    else
        echo "   ⚠️  Still no map publisher. Trying manual map trigger..."
        
        # Try to echo map data to see if it's there
        echo "   🔍 Testing map data availability..."
        timeout 3s ros2 topic echo /map --once > /tmp/map_test.txt 2>&1
        
        if [ -s /tmp/map_test.txt ]; then
            echo "   ✅ Map data is available!"
        else
            echo "   ❌ No map data available. SLAM might not have generated a map yet."
            echo ""
            echo "💡 Solutions:"
            echo "   1. Move your lidar around more to generate map data"
            echo "   2. Wait longer for SLAM to process the data"
            echo "   3. Check SLAM logs for errors"
            echo ""
            read -p "Do you want to try saving anyway? (y/n): " -n 1 -r
            echo ""
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
        fi
        rm -f /tmp/map_test.txt
    fi
else
    echo "   ❌ Map topic not found! Make sure SLAM is running."
    exit 1
fi

echo ""
echo "💾 Saving map as: $MAP_NAME"
echo "📁 Map directory: $MAP_DIR"
echo ""

# Save the map using multiple methods
cd "$MAP_DIR"

echo "💾 Attempting to save map using Method 1: map_saver_cli..."
timeout 15s ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" --ros-args -p save_map_timeout:=15000 2>&1

# Check if first method worked
if [ -f "${MAP_NAME}.pgm" ] && [ -f "${MAP_NAME}.yaml" ]; then
    echo "✅ Method 1 successful!"
else
    echo "⚠️  Method 1 failed, trying Method 2: SLAM toolbox service..."
    
    # Try using SLAM toolbox service
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '${MAP_DIR}/${MAP_NAME}'}}" 
    sleep 3
    
    if [ -f "${MAP_NAME}.pgm" ] && [ -f "${MAP_NAME}.yaml" ]; then
        echo "✅ Method 2 successful!"
    else
        echo "⚠️  Method 2 failed, trying Method 3: Direct topic capture..."
        
        # Try capturing map data directly and converting
        echo "📡 Capturing map data directly from topic..."
        timeout 10s ros2 topic echo /map --once > "${MAP_NAME}_raw.txt" 2>&1
        
        if [ -s "${MAP_NAME}_raw.txt" ]; then
            echo "✅ Map data captured, but manual conversion needed"
            echo "📄 Raw map data saved as: ${MAP_NAME}_raw.txt"
        else
            echo "❌ All methods failed!"
            echo ""
            echo "🔧 Troubleshooting steps:"
            echo "   1. Make sure you've moved the lidar around to generate a map"
            echo "   2. Wait a few more seconds for SLAM to process"
            echo "   3. Try: ros2 topic echo /map --once (to see if data exists)"
            echo "   4. Check SLAM logs for errors"
            echo ""
            echo "Current SLAM status:"
            ros2 node list | grep slam || echo "No SLAM nodes found"
            exit 1
        fi
    fi
fi

echo ""
if [ -f "${MAP_NAME}.pgm" ] && [ -f "${MAP_NAME}.yaml" ]; then
    echo "✅ Map saved successfully!"
    echo ""
    echo "📄 Files created:"
    echo "   • ${MAP_DIR}/${MAP_NAME}.pgm  (map image)"
    echo "   • ${MAP_DIR}/${MAP_NAME}.yaml (map metadata)"
    echo ""
    echo "📊 Map information:"
    ls -lh "${MAP_NAME}.pgm" "${MAP_NAME}.yaml"
    echo ""
    echo "🎯 To view this map:"
    echo "   ./view_map.sh $MAP_NAME"
else
    echo "❌ Map saving failed!"
    echo "Check if SLAM system is publishing to /map topic"
fi
echo "  - $MAP_DIR/$MAP_NAME.yaml (map metadata)"
echo "  - $MAP_DIR/$MAP_NAME.pgm (map image)"

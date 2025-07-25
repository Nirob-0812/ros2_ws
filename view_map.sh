#!/bin/bash

# View Saved Map in RViz
# ======================

echo "🗺️  Map Viewer"
echo "=============="
echo ""

source ~/ros2_ws/install/setup.bash

MAP_DIR="$HOME/ros2_ws/maps"

# Check if map name is provided
if [ -z "$1" ]; then
    echo "Available maps:"
    ls -1 "$MAP_DIR"/*.yaml 2>/dev/null | sed 's/.*\///g' | sed 's/.yaml//g' | nl
    echo ""
    echo "Usage: ./view_map.sh <map_name>"
    echo "Example: ./view_map.sh house_map_20250724_145900"
    exit 1
fi

MAP_NAME=$1
MAP_FILE="$MAP_DIR/${MAP_NAME}.yaml"

# Check if map exists
if [ ! -f "$MAP_FILE" ]; then
    echo "❌ Map file not found: $MAP_FILE"
    echo ""
    echo "Available maps:"
    ls -1 "$MAP_DIR"/*.yaml 2>/dev/null | sed 's/.*\///g' | sed 's/.yaml//g' || echo "No maps found"
    exit 1
fi

echo "📁 Loading map: $MAP_NAME"
echo "📄 Map file: $MAP_FILE"
echo ""

# Kill any existing processes
echo "🧹 Cleaning up existing processes..."
pkill -f "map_server" 2>/dev/null
pkill -f "rviz" 2>/dev/null
sleep 2

echo "🚀 Starting map server..."
# Start map server to publish the saved map
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP_FILE" -p topic_name:="map" -p frame_id:="map" &
MAP_SERVER_PID=$!

# Wait for map server to start
echo "⏳ Waiting for map server to start..."
sleep 3

# Check if map server is running
if ! kill -0 $MAP_SERVER_PID 2>/dev/null; then
    echo "❌ Map server failed to start!"
    exit 1
fi

echo "✅ Map server started successfully!"
echo ""

# Start static transform for the map frame
echo "🔗 Starting transform publisher..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
TRANSFORM_PID=$!

echo "🖥️  Starting RViz..."
echo ""
echo "📋 In RViz:"
echo "   1. Set Fixed Frame to 'map'"
echo "   2. Add Map display and set topic to '/map'"
echo "   3. Your saved map should appear!"
echo ""
echo "Press Ctrl+C to stop viewing"
echo ""

# Start RViz with map viewing configuration
rviz2 &
RVIZ_PID=$!

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🧹 Cleaning up..."
    kill $MAP_SERVER_PID 2>/dev/null
    kill $TRANSFORM_PID 2>/dev/null
    kill $RVIZ_PID 2>/dev/null
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for user to press Ctrl+C
wait $RVIZ_PID

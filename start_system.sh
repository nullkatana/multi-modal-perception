#!/bin/bash
# Multi-Sensor Object Detection System - Launch Script
# Starts all ROS 2 nodes for the sensor fusion system

echo "================================================"
echo "  Multi-Sensor Object Detection System"
echo "  Starting all nodes..."
echo "================================================"

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "ERROR: Virtual environment not found!"
    echo "Please run: python3 -m venv venv"
    exit 1
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Source ROS 2
echo "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down all nodes..."
    kill 0
    wait
    echo "All nodes stopped."
}

# Register cleanup function
trap cleanup EXIT INT TERM

# Start all nodes in background
echo ""
echo "Starting Shared Object State..."
python3 src/shared_object_state.py &
PID_STATE=$!

echo "Starting IR Sensor Simulator..."
python3 src/ir_sensor_simulator.py &
PID_IR=$!

echo "Starting NIR Sensor Simulator..."
python3 src/nir_sensor_simulator.py &
PID_NIR=$!

echo "Starting ToF Array Simulator..."
python3 src/tof_array_simulator.py &
PID_TOF=$!

echo "Starting LiDAR Simulator..."
python3 src/lidar_simulator.py &
PID_LIDAR=$!

echo "Starting Point Cloud Generator..."
python3 src/pointcloud_generator.py &
PID_PC=$!

echo "Starting Object Detector (2-sensor)..."
python3 src/object_detector.py &
PID_DETECTOR=$!

echo "Starting Multi-Sensor Fusion..."
python3 src/multi_sensor_fusion.py &
PID_FUSION=$!

echo "Starting Object Tracker..."
python3 src/object_tracker.py &
PID_TRACKER=$!

echo "Starting Trajectory Visualizer..."
python3 src/trajectory_visualizer.py &
PID_VIZ=$!

echo "Starting TF Broadcaster..."
python3 src/tf_broadcaster.py &
PID_TF=$!

echo "Starting Metrics Logger..."
python3 src/metrics_logger.py &
PID_METRICS=$!

# Wait a moment for nodes to start
sleep 3

echo ""
echo "================================================"
echo "  All nodes started successfully!"
echo "================================================"
echo ""
echo "Running processes:"
echo "  - Shared Object State (PID: $PID_STATE)"
echo "  - IR Sensor (PID: $PID_IR)"
echo "  - NIR Sensor (PID: $PID_NIR)"
echo "  - ToF Array (PID: $PID_TOF)"
echo "  - LiDAR (PID: $PID_LIDAR)"
echo "  - Point Cloud Generator (PID: $PID_PC)"
echo "  - Object Detector (PID: $PID_DETECTOR)"
echo "  - Multi-Sensor Fusion (PID: $PID_FUSION)"
echo "  - Object Tracker (PID: $PID_TRACKER)"
echo "  - Trajectory Visualizer (PID: $PID_VIZ)"
echo "  - TF Broadcaster (PID: $PID_TF)"
echo "  - Metrics Logger (PID: $PID_METRICS)"
echo ""
echo "Data Sources (4 sensors + scene representation):"
echo "  Sensors:"
echo "    • IR (single-point, 10Hz)"
echo "    • NIR (single-point, 12Hz)"
echo "    • ToF Array (8x8 grid, 20Hz)"
echo "    • LiDAR (360° scan, 10Hz)"
echo "  Scene:"
echo "    • Point Cloud (3D scene, 5Hz)"
echo ""
echo "To visualize, run in another terminal:"
echo "  cd ~/multi_modal_perception"
echo "  source venv/bin/activate"
echo "  source /opt/ros/humble/setup.bash"
echo "  rviz2"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "================================================"

# Wait for all background processes
wait
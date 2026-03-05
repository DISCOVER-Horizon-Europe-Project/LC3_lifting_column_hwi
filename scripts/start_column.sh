#!/bin/bash
# Quick start script for LC3 column controller

echo "LC3 Lifting Column - Quick Start"
echo "================================="
echo ""

# Check if sourced
if [ -f "$HOME/ros2_yeray/install/setup.bash" ]; then
    source $HOME/ros2_yeray/install/setup.bash
    echo "✓ ROS2 workspace sourced"
else
    echo "✗ Error: Cannot find ROS2 workspace at $HOME/ros2_yeray"
    exit 1
fi

echo ""
echo "Starting LC3 column controller..."
echo "Press Ctrl+C to stop (column will automatically retract to 0)"
echo ""

# Launch with passed arguments or default
ros2 launch lc3_hw_interface lc3_column.launch.py "$@"

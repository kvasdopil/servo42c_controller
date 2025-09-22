#!/bin/bash

set -e  # Exit on error

. /opt/ros/jazzy/setup.bash

# Navigate to the workspace root
cd "$(dirname "$0")/.."
WORKSPACE_DIR=$(pwd)

# Create workspace if it doesn't exist
if [ ! -f "$WORKSPACE_DIR/servo42c_controller/package.xml" ]; then
    echo "Error: servo42c_controller package not found in src directory."
    echo "This script should be run from within the servo42c_controller directory."
    exit 1
fi

# Build packages
echo "Building packages..."
colcon build --symlink-install --packages-select servo42c_controller

# Source setup
. install/setup.bash

# Run the servo controller (minimal launch without MoveIt)
echo "Launching servo controller (minimal)..."
ros2 launch servo42c_controller minimal.launch.py 

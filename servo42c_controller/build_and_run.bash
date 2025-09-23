#!/bin/bash

set -e  # Exit on error

# Set environment variables to avoid unbound variable errors
export AMENT_TRACE_SETUP_FILES=
export COLCON_TRACE=
export AMENT_PYTHON_EXECUTABLE=python3
export COLCON_PREFIX_PATH=

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

# Run the servo controller (main launch with MoveIt)
echo "Launching servo controller..."
ros2 launch servo42c_controller main.launch.py 

#!/bin/bash

# Octane system startup script
# Main entry point for launching all packcages

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

echo "=== Starting Octane System ==="
echo "Workspace: ${WORKSPACE_ROOT}"
echo ""

# Clean workspace
echo "=== Cleaning Workspace ==="
cd "${WORKSPACE_ROOT}"
rm -rf build install log
echo "[OK] Workspace cleaned"
echo ""

# Build required packages
echo "=== Building Packages ==="
cd "${WORKSPACE_ROOT}"

# System Package
    # octane
# Perception packages: 
    # orbbec_camera_msgs
    # orbbec_camera
    # octane_perception
MAKEFLAGS="-j2" colcon build --packages-select \
    orbbec_camera_msgs orbbec_camera octane_perception \
    octane \
    --parallel-workers 1

echo "[OK] Packages built"
echo ""

# Source ROS2 and workspace
if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
    source "${WORKSPACE_ROOT}/install/setup.bash"
    echo "[OK] Workspace sourced"
else
    echo "[ERROR] Failed to source workspace"
    exit 1
fi

# Launch perception subsystem
echo ""
echo "=== Launching Perception Subsystem ==="
ros2 launch octane perception.launch.py &
PERCEPTION_PID=$!

# Wait for user interrupt
echo ""
echo "=== Octane System Running ==="
echo "Press Ctrl+C to shutdown"
echo ""

trap 'echo ""; echo "=== Shutting Down Octane System ==="; kill $PERCEPTION_PID 2>/dev/null; exit 0' SIGINT SIGTERM

wait

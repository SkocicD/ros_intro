#!/bin/bash

# Kill ROS2 Docker container

set -e

echo "=== Stopping ROS2 Container ==="

# Stop the container
if docker ps -q -f name=ros2 | grep -q .; then
    echo "Stopping ros2 container..."
    docker stop ros2
    echo "[OK] Container stopped"
else
    echo "[INFO] Container not running"
fi

# Optional: Remove the container to start fresh next time
# Uncomment if you want to remove the container completely
# docker rm ros2 2>/dev/null && echo "[OK] Container removed"

echo ""
echo "Container stopped. Use ./run_ros.sh to restart."

#!/bin/bash

# Build Docker container with VNC and desktop environment

echo "=== Building ROS2 Docker Container ==="
echo ""
echo "This will build a Docker image with:"
echo "  - ROS2 Humble"
echo "  - XFCE4 Desktop Environment"
echo "  - TigerVNC Server"
echo "  - ROS Visualization Tools (rqt, rviz2)"
echo ""
echo "This may take 15-30 minutes depending on your system..."
echo ""

# Build with progress output
docker compose build --progress=plain

if [ $? -eq 0 ]; then
    echo ""
    echo "=== BUILD SUCCESSFUL ==="
    echo ""
    echo "Next steps:"
    echo "  1. Start container: docker compose up -d"
    echo "  2. Start VNC: ./start_vnc.sh"
    echo "  3. Connect to: vnc://localhost:5901 (password: octane)"
    echo ""
else
    echo ""
    echo "=== BUILD FAILED ==="
    echo "Check the output above for errors."
    echo ""
    exit 1
fi

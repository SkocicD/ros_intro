#!/bin/bash

# Start VNC server in Docker container
# This runs in the foreground - press Ctrl+C to stop

echo "=== Starting VNC Server ==="
echo "Password: octane"
echo ""
echo "Connect with VNC viewer:"
echo "  Address: vnc://localhost:5901"
echo "  Password: octane"
echo ""
echo "VNC server will run in the foreground."
echo "Press Ctrl+C to stop the VNC server."
echo ""
echo "========================================="
echo ""

# Start VNC server (not in background)
docker exec -it ros2 bash -c "vncserver :1 -geometry 1920x1080 -depth 24 -fg"

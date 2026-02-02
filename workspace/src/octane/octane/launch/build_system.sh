#!/bin/bash

# Build all Octane system packages

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

echo "=== Building Octane System ==="
echo "Workspace: ${WORKSPACE_ROOT}"
echo ""

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

echo ""
echo "[OK] Build complete"

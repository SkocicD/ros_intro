#!/bin/bash

# Clean all build artifacts from workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

echo "=== Cleaning Workspace ==="
echo "Workspace: ${WORKSPACE_ROOT}"
echo ""

cd "${WORKSPACE_ROOT}"

# Remove build artifacts
if [ -d "build" ]; then
    echo "Removing build/"
    rm -rf build
fi

if [ -d "install" ]; then
    echo "Removing install/"
    rm -rf install
fi

if [ -d "log" ]; then
    echo "Removing log/"
    rm -rf log
fi

echo ""
echo "[OK] Workspace cleaned"
echo "Run './launch_system.sh' to rebuild and launch"

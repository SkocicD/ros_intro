#!/bin/bash

# Install dependencies for perception packages

set -e

echo "=== Installing Perception Dependencies ==="

apt-get update

# Orbbec camera dependencies
apt-get install -y \
  ros-humble-backward-ros \
  ros-humble-camera-info-manager \
  ros-humble-cv-bridge \
  ros-humble-diagnostic-updater \
  ros-humble-image-publisher \
  ros-humble-image-transport \
  ros-humble-rclcpp-components \
  ros-humble-tf2-sensor-msgs \
  libgflags-dev \
  nlohmann-json3-dev \
  libgoogle-glog-dev \
  libssl-dev

echo "[OK] Perception dependencies installed"

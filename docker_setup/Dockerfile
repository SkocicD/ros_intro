FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
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
    libssl-dev \
    python3-flask \
    python3-opencv \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace


# ═══════════════════════════════════════════════════════════════════════════
# Madara 6DoF Arm — Docker image
# Base: ros:humble-ros-base-jammy (Ubuntu 22.04 + ROS2 Humble)
#       ↑ Official multi-arch image: linux/amd64 + linux/arm64/v8
#       osrf/ros:humble-desktop-full is amd64-only — unusable on RPi 5
# Camera: camera_ros (libcamera backend) — correct for Raspberry Pi 5 + IMX219
# Arch:   auto-selected from manifest — no flags needed
# ═══════════════════════════════════════════════════════════════════════════
FROM ros:humble-ros-base-jammy

# ── System + ROS packages ──────────────────────────────────────────────────
# RViz2 and Gazebo Fortress added explicitly since we're not on desktop-full
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control \
    ros-humble-moveit \
    ros-humble-rviz2 \
    ros-humble-camera-ros \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    libcamera-dev \
    libcamera-tools \
    setserial \
    nano \
    curl \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# ── rosdep init ────────────────────────────────────────────────────────────
RUN apt-get update && \
    rosdep init 2>/dev/null || true && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

# ── Group memberships ──────────────────────────────────────────────────────
# dialout → UART access (/dev/ttyAMA0, /dev/ttyUSB0)
# video   → /dev/video*, /dev/dma_heap/ (camera DMA buffers on RPi 5)
# render  → /dev/dma_heap/linux,cma (required on RPi 5 for libcamera)
RUN usermod -aG dialout root 2>/dev/null || true && \
    usermod -aG video   root 2>/dev/null || true && \
    usermod -aG render  root 2>/dev/null || true

# ── Workspace skeleton ─────────────────────────────────────────────────────
RUN mkdir -p /madara_ws/src

# ── Entrypoint ─────────────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# ── Shell convenience ──────────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /madara_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

WORKDIR /madara_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

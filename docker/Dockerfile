# Dev/sim image for the full semantic navigation stack. amd64 only:
# Gazebo Classic has no arm64 binaries (Ubuntu jammy doesn't package it
# and the ROS repo builds gazebo-ros-pkgs/turtlebot3-gazebo for amd64
# only), so on Apple Silicon this runs under Rosetta 2 — pinned via
# `platform: linux/amd64` in compose.yaml. The base image provides an
# Ubuntu 22.04 + ROS Humble desktop reachable in a browser via noVNC
# (port 80 in-container).
FROM tiryoh/ros2-desktop-vnc:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-tf2-geometry-msgs \
    python3-pip \
    curl \
    && rm -rf /var/lib/apt/lists/*

# CPU torch first, or ultralytics pulls the CUDA build. Pins applied
# last: numpy 2 breaks the apt cv_bridge (compiled against 1.x), and
# newer setuptools breaks colcon --symlink-install on Humble.
RUN pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir ultralytics \
    && pip3 install --no-cache-dir 'numpy<2' 'setuptools==58.2.0'

# Software GL: no GPU passthrough under Docker Desktop on macOS.
ENV TURTLEBOT3_MODEL=waffle_pi \
    LIBGL_ALWAYS_SOFTWARE=1

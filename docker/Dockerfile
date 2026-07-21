# Gazebo Classic has no arm64 binaries, so this image is amd64-only;
# compose pins the platform and Apple Silicon runs it under Rosetta 2.
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

# numpy<2: apt cv_bridge is built against 1.x.
# setuptools 58.2.0: newer breaks colcon --symlink-install on Humble.
RUN pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir ultralytics \
    && pip3 install --no-cache-dir 'numpy<2' 'setuptools==58.2.0'

ENV TURTLEBOT3_MODEL=waffle_pi \
    LIBGL_ALWAYS_SOFTWARE=1

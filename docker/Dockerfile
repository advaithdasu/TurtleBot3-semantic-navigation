# GPU sim image: ROS 2 Jazzy (Ubuntu Noble) + Gazebo Harmonic + CUDA torch,
# driven from JupyterLab.
#
# Base choice — plain ubuntu:24.04, not nvidia/cuda:*
# ---------------------------------------------------
# Nothing here compiles CUDA code, so we need no nvcc and no CUDA headers.
# The two things we *do* need both come from elsewhere:
#   * compute — the pip torch cu124 wheels vendor their own CUDA runtime
#     (nvidia-cublas-cu12, nvidia-cudnn-cu12, ...), so libcudart never has
#     to come from the base image.
#   * graphics — libEGL_nvidia.so and its ICD json are injected at *run*
#     time by the NVIDIA Container Toolkit, keyed off NVIDIA_VISIBLE_DEVICES
#     and NVIDIA_DRIVER_CAPABILITIES (set below), not off the base image.
# Skipping the CUDA base saves ~2 GB and removes a tag-pinning maintenance
# burden. Add `nvidia/cuda:...-devel-*` back only if something starts
# needing nvcc.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Instructs the NVIDIA Container Toolkit what to inject. `graphics` and
# `display` are the load-bearing ones and are NOT in the usual
# "compute,utility" default: without them gz-sim's ogre2 finds no NVIDIA
# EGL vendor and silently falls back to llvmpipe, which is the exact
# software-rendering bottleneck this image exists to escape.
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# ── APT repos: ROS 2 Jazzy + Gazebo Harmonic ─────────────────────────────
# Harmonic is not in the ROS repo; ros-jazzy-ros-gz-sim depends on the
# gz-harmonic packages served by packages.osrfoundation.org.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 + Gazebo + the GL/EGL loader stack ─────────────────────────────
# Notes on the less obvious entries:
#
#   ros-jazzy-desktop — deliberately the full variant, not ros-base. The
#     workspace has two ament_cmake packages whose build tooling is not in
#     ros-base: tb3_query needs ament_cmake_python and
#     rosidl_default_generators (it generates SemanticQueryResult.msg), and
#     tb3_frontier_exploration does find_package(ament_lint_auto REQUIRED)
#     under the default BUILD_TESTING=ON. Trimming to ros-base means
#     re-adding those four by hand for a colcon failure deep inside a CMake
#     configure step. rviz2 comes along too, which is useful over X
#     forwarding even though the container itself is headless.
#
#   libglvnd0/libegl1/libgles2 — the *dispatch* libraries. The actual
#     NVIDIA implementations arrive at runtime via the container toolkit;
#     without the loaders present that injection has nothing to bind into.
#     libgl1-mesa-dri stays as a deliberate llvmpipe fallback so the image
#     is still usable (slowly) on a box with no GPU.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-desktop \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-image \
        ros-jazzy-turtlebot3-gazebo \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-vision-msgs \
        ros-jazzy-cv-bridge \
        ros-jazzy-tf2-geometry-msgs \
        python3-colcon-common-extensions \
        python3-pip \
        libglvnd0 libgl1 libglx0 libegl1 libgles2 libgl1-mesa-dri \
        libxext6 libx11-6 \
        xvfb \
        git wget \
    && rm -rf /var/lib/apt/lists/*

# ── Python: CUDA torch, YOLO, JupyterLab ─────────────────────────────────
# torch comes from the cu124 index. This is the inverse of the previous
# arm64 image, which pulled from the CPU index and asserted that *no*
# nvidia-* wheels were present — on this host they are exactly what we
# want, so that guard is gone and replaced by the CUDA-build assert below.
#
# Pins, all still load-bearing:
#   * numpy<2      — Jazzy's apt cv_bridge is built against Noble's 1.26,
#                    and a pip numpy 2.x in dist-packages shadows it.
#   * setuptools<80 — colcon-core requires it; installing torch otherwise
#                    pulls setuptools 83 and breaks colcon build.
#   * opencv-python<5 — ultralytics pulls opencv-python unpinned and it now
#                    resolves to 5.x. OpenCV 5 renumbers the CV_8UC* type
#                    constants (CV_8UC3 went 16 → 64) while ros-jazzy
#                    cv_bridge's compiled extension still returns the
#                    OpenCV-4 numbering, so cv_bridge raises KeyError(16)
#                    on conversion. Asserted directly so a future OpenCV
#                    bump fails the build instead of the runtime.
# All three must be installed last, after anything that might upgrade them.
#
# Noble marks its Python environment externally managed (PEP 668), hence
# --break-system-packages: ROS nodes run against the system interpreter,
# so a venv would be invisible to them.
RUN pip3 install --no-cache-dir --break-system-packages \
        --index-url https://download.pytorch.org/whl/cu124 torch torchvision \
    && pip3 install --no-cache-dir --break-system-packages \
        ultralytics jupyterlab matplotlib ipywidgets \
    && pip3 install --no-cache-dir --break-system-packages \
        'numpy<2' 'setuptools<80' 'opencv-python<5' \
    && python3 -c "import torch; assert torch.version.cuda, 'torch is a CPU build; expected a cu12x wheel'; print('torch', torch.__version__, 'cuda', torch.version.cuda)" \
    && python3 -c "import cv2; assert cv2.CV_8UC3 == 16, f'CV_8UC3={cv2.CV_8UC3}, expected 16 (cv_bridge ABI mismatch)'" \
    && python3 -c "import numpy; print('numpy', numpy.__version__)"

# NOTE: unlike the previous CPU image, the packaged waffle_pi SDF is left
# stock. The old build sed'ed camera 30→10 Hz, IMU 200→50 Hz and disabled
# sensor <visualize> purely because gz-sim rendered through llvmpipe on
# macOS. With the camera rendering on an actual GPU those workarounds cost
# fidelity for no gain, so the sensors run at their upstream rates.

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENV TURTLEBOT3_MODEL=waffle_pi

# Pre-create every path compose mounts a *named* volume onto, owned by
# `ubuntu`. Docker seeds a fresh named volume from the image's content at
# that path, ownership included — but if the path does not exist in the
# image it creates an empty root-owned directory instead, and the first
# `colcon build` then dies with EACCES on build/. Creating them here is
# what makes the non-root user work with the volume layout in
# compose.yaml; keep the two in sync.
RUN mkdir -p /home/ubuntu/ws/build /home/ubuntu/ws/install /home/ubuntu/ws/log \
             /home/ubuntu/.tb3_semantic_nav/evidence \
    && chown -R ubuntu:ubuntu /home/ubuntu

# uid 1000 `ubuntu` ships with Noble. Running as it (rather than root)
# keeps files written into the bind-mounted workspace owned by the host
# user instead of root.
USER ubuntu
WORKDIR /home/ubuntu/ws

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
# Token auth is left ON. Jupyter reads JUPYTER_TOKEN from the environment
# (compose sets it); with it unset, Jupyter generates one and prints the
# URL to the container log. Do not add `--ServerApp.token=` here — the
# grounding server on 8801 is already unauthenticated, and an open
# notebook on a box with a public IP is a root shell for anyone who finds
# it. compose publishes both ports on 127.0.0.1 only; reach them over an
# SSH tunnel.
CMD ["jupyter", "lab", "--ip=0.0.0.0", "--port=8888", "--no-browser", \
     "--ServerApp.root_dir=/home/ubuntu/ws"]

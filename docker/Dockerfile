# Native arm64 sim image: ROS 2 Jazzy (Ubuntu Noble) + Gazebo Harmonic.
#
# The previous image was pinned linux/amd64 and ran under Rosetta 2 purely
# because Gazebo Classic has no arm64 Linux binaries. Harmonic (gz-sim 8)
# ships arm64 debs, and Jazzy's turtlebot3_gazebo is already gz-sim based,
# so nothing here is emulated any more — compose no longer sets `platform`.
FROM tiryoh/ros2-desktop-vnc:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
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
    python3-pip \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Trim sensor rates on the packaged waffle_pi SDF. Rendering is still the
# bottleneck — Docker on macOS has no GPU passthrough, so gz-sim's ogre2
# runs on llvmpipe — while physics/SLAM/Nav2 now run at native speed:
# camera 30 → 10 Hz (detector caps inference at 5 Hz anyway; 640x480 kept),
# IMU 200 → 50 Hz (nothing consumes /imu), and lidar/camera <visualize>
# off (the GUI's sensor overlays re-render every frame). Seds are scoped
# to the specific <sensor> blocks so the DiffDrive/JointStatePublisher
# plugin settings are untouched; the asserts fail the build if upstream
# renames a sensor and a sed silently no-ops.
RUN SDF=/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
    && sed -i '/<sensor name="camera"/,/<\/sensor>/ s|<update_rate>30</update_rate>|<update_rate>10</update_rate>|' "$SDF" \
    && sed -i '/<sensor name="camera"/,/<\/sensor>/ s|<visualize>true</visualize>|<visualize>false</visualize>|' "$SDF" \
    && sed -i '/<sensor name="hls_lfcd_lds"/,/<\/sensor>/ s|<visualize>true</visualize>|<visualize>false</visualize>|' "$SDF" \
    && sed -i '/<sensor name="tb3_imu"/,/<\/sensor>/ s|<update_rate>200</update_rate>|<update_rate>50</update_rate>|' "$SDF" \
    && [ "$(grep -c '<update_rate>10</update_rate>' "$SDF")" = "2" ] \
    && [ "$(grep -c '<update_rate>50</update_rate>' "$SDF")" = "1" ] \
    && [ "$(grep -c '<update_rate>30</update_rate>' "$SDF")" = "0" ] \
    && [ "$(grep -c '<visualize>true</visualize>' "$SDF")" = "0" ]

# torch/torchvision come from download.pytorch.org's CPU index, which does
# publish linux aarch64 wheels (manylinux_2_28_aarch64). Do NOT take torch
# from plain PyPI here: its aarch64 wheels are CUDA-enabled (built for
# Grace/sbsa) and drag in ~20 nvidia-* packages plus cuda-toolkit and
# triton — gigabytes of libraries that cannot be used on a Mac.
#
# Pins, all load-bearing:
#   * numpy<2      — Jazzy's apt cv_bridge is built against Noble's 1.26,
#                    and a pip numpy 2.x in dist-packages shadows it.
#   * setuptools<80 — colcon-core 0.21.0 requires it; installing torch
#                    otherwise pulls setuptools 83 and breaks colcon build.
#                    (The old Humble image pinned 58.2.0; that predates
#                    Python 3.12 and must not come back.)
#   * opencv-python<5 — ultralytics pulls in opencv-python unpinned, and it
#                    now resolves to 5.x. OpenCV 5 renumbers the CV_8UC*
#                    family of type constants (CV_8UC3 went from 16 to 64);
#                    ros-jazzy-cv-bridge's compiled boost extension still
#                    returns the OpenCV-4 numbering, so cv_bridge's
#                    cvtype_to_name[cv_type] lookup in cv2_to_imgmsg()
#                    raises KeyError(16) on every call. Symptom: detections
#                    publish fine (no cv_bridge involved) but every
#                    ~/debug_image publish fails, so RViz's Image panel and
#                    grounding/eval capture stay blank with no error visible
#                    outside the node's log. Assert the constant directly so
#                    a future OpenCV bump fails the build instead of the
#                    runtime.
# All three must be installed last, after anything that might upgrade them.
#
# Noble marks its Python environment as externally managed (PEP 668), hence
# --break-system-packages: ROS nodes run against the system interpreter, so
# a venv would be invisible to them.
RUN pip3 install --no-cache-dir --break-system-packages \
        --index-url https://download.pytorch.org/whl/cpu torch torchvision \
    && pip3 install --no-cache-dir --break-system-packages ultralytics \
    && pip3 install --no-cache-dir --break-system-packages 'numpy<2' 'setuptools<80' 'opencv-python<5' \
    && python3 -c "import torch, numpy, ultralytics; print('torch', torch.__version__, 'numpy', numpy.__version__)" \
    && python3 -c "import cv2; assert cv2.CV_8UC3 == 16, f'CV_8UC3={cv2.CV_8UC3}, expected 16 (cv_bridge ABI mismatch)'" \
    && pip3 list 2>/dev/null | grep -iE '^setuptools |^colcon-core |^opencv-python ' \
    # Guard against silently regressing to plain PyPI's CUDA-enabled aarch64
    # torch. nvidia-ml-py is allowed: ultralytics pulls it for GPU monitoring
    # and it is pure-Python NVML bindings, not a CUDA runtime library.
    && [ -z "$(pip3 list 2>/dev/null | awk '{print $1}' \
        | grep -iE '^nvidia-|^cuda-|^triton$' | grep -ivE '^nvidia-ml-py$')" ]

ENV TURTLEBOT3_MODEL=waffle_pi \
    LIBGL_ALWAYS_SOFTWARE=1

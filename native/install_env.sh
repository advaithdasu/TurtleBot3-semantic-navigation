#!/usr/bin/env bash
# One-time environment setup for hosts with NO Docker and NO root — e.g. a
# JupyterHub/SystemdSpawner GPU cluster where sudo is locked down
# (NoNewPrivileges) and there is no Apptainer/Podman either.
#
# Installs ROS 2 Jazzy + Gazebo Harmonic entirely via conda packages from
# RoboStack (https://robostack.github.io/), which ship prebuilt binaries
# that need no apt/root at all — everything lands under
# ~/micromamba/envs/tb3. See ../docs/NATIVE_SETUP.md for the full picture.
#
#   ./native/install_env.sh
#
# Safe to re-run: micromamba create -y overwrites the env in place.
set -euo pipefail

ENV_NAME="tb3"
MAMBA_ROOT="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"

echo "=== 1. micromamba ==="
if ! command -v micromamba >/dev/null 2>&1; then
  export MAMBA_ROOT_PREFIX="${MAMBA_ROOT}"
  "${SHELL}" <(curl -L micro.mamba.pm) < /dev/null
  export PATH="${HOME}/.local/bin:${PATH}"
fi
if ! command -v micromamba >/dev/null 2>&1; then
  echo "error: micromamba install finished but 'micromamba' is not on PATH." >&2
  echo "       Open a new shell (or 'source ~/.bashrc') and re-run this script." >&2
  exit 1
fi
micromamba --version

echo
echo "=== 2. ${ENV_NAME} conda env (ROS 2 Jazzy + Gazebo Harmonic, RoboStack) ==="
# robostack-jazzy first (strict priority) so its pins win over conda-forge's
# own, generally newer, builds of the same library names.
micromamba create -y -n "${ENV_NAME}" \
  -c robostack-jazzy -c conda-forge \
  --strict-channel-priority \
  python=3.12 \
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
  ros-jazzy-ament-lint-auto \
  ros-jazzy-ament-lint-common \
  colcon-common-extensions \
  compilers cmake ninja pkg-config make \
  libgl-devel

echo
echo "=== 3. Python deps (torch/CUDA, YOLO, Jupyter) ==="
# Same pins as docker/Dockerfile, and for the same reasons:
#   - opencv-python<5: ultralytics pulls an unpinned opencv-python wheel
#     that resolves to 5.x, which renumbers CV_8UC* constants (CV_8UC3
#     16 -> 64) relative to what ros-jazzy-cv-bridge's compiled extension
#     expects, and raises KeyError(16) at conversion time.
# torch comes from the cu124 wheel index, which vendors its own CUDA
# runtime — no nvcc/CUDA toolkit needed on the host, only the driver
# (already present: nvidia-smi works outside any container here).
micromamba run -n "${ENV_NAME}" pip install --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cu124 torch torchvision
micromamba run -n "${ENV_NAME}" pip install --no-cache-dir \
    ultralytics jupyterlab ipykernel matplotlib ipywidgets 'opencv-python<5'

micromamba run -n "${ENV_NAME}" python3 -c \
  "import torch; assert torch.version.cuda, 'torch is a CPU build; expected a cu12x wheel'; print('torch', torch.__version__, 'cuda', torch.version.cuda)"
micromamba run -n "${ENV_NAME}" python3 -c \
  "import cv2; assert cv2.CV_8UC3 == 16, f'CV_8UC3={cv2.CV_8UC3}, expected 16 (cv_bridge ABI mismatch)'"

echo
echo "=== 4. Register a Jupyter kernel for this env ==="
# Lands in ~/.local/share/jupyter/kernels/tb3 — the JupyterHub single-user
# server picks it up from there with no restart, just a fresh launcher tab.
micromamba run -n "${ENV_NAME}" python3 -m ipykernel install --user \
  --name "${ENV_NAME}" --display-name "TB3 Semantic Nav (RoboStack)"

echo
echo "=== Done ==="
echo "Next: ./native/setup_ws.sh   (downloads YOLO weights, builds the workspace)"
echo "Every session: micromamba activate ${ENV_NAME}"

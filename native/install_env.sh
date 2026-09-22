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
BIN_DIR="${HOME}/.local/bin"
if ! command -v micromamba >/dev/null 2>&1; then
  # The official installer (micro.mamba.pm) is interactive (prompts for
  # install dir, shell init, ...) and piping /dev/null at it doesn't
  # reliably answer those prompts in a non-interactive shell. Fetch the
  # static binary directly instead — no prompts, nothing to get wrong.
  mkdir -p "${BIN_DIR}"
  ARCH="$(uname -m)"
  case "${ARCH}" in
    x86_64) PLATFORM="linux-64" ;;
    aarch64|arm64) PLATFORM="linux-aarch64" ;;
    *) echo "error: unrecognized architecture '${ARCH}' — see" \
            "https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html" >&2
       exit 1 ;;
  esac
  curl -Ls "https://micro.mamba.pm/api/micromamba/${PLATFORM}/latest" \
    | tar -xj -C "${BIN_DIR}" --strip-components=1 bin/micromamba
  chmod +x "${BIN_DIR}/micromamba"
fi
export PATH="${BIN_DIR}:${PATH}"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT}"

if ! command -v micromamba >/dev/null 2>&1; then
  echo "error: expected micromamba at ${BIN_DIR}/micromamba but it's not on PATH." >&2
  exit 1
fi
micromamba --version

# Make this permanent for future shells/terminals (JupyterHub terminals
# start a fresh shell each time), and make it idempotent to re-run.
for rc in "${HOME}/.bashrc" "${HOME}/.bash_profile"; do
  [ -f "${rc}" ] || continue
  grep -q "MAMBA_ROOT_PREFIX=.*micromamba" "${rc}" 2>/dev/null && continue
  {
    echo ""
    echo "# added by TurtleBot3-semantic-navigation/native/install_env.sh"
    echo "export MAMBA_ROOT_PREFIX=\"${MAMBA_ROOT}\""
    echo "export PATH=\"${BIN_DIR}:\${PATH}\""
  } >> "${rc}"
  echo "Added micromamba to PATH in ${rc} — open a new terminal (or 'source ${rc}') to pick it up there."
done

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
#   - setuptools<80: setuptools 80.0.0 dropped the legacy `develop
#     --editable` flag that colcon's --symlink-install uses to build
#     ament_python packages (pypa/setuptools#4971), so anything installed
#     here that upgrades setuptools past 80 breaks `colcon build
#     --symlink-install` with "error: option --editable not recognized".
#     Confirmed hitting ROS 2 Jazzy specifically (ros2/ros2#1702); 79.0.1
#     is the version people report as working, hence the pin below.
# torch comes from the cu124 wheel index, which vendors its own CUDA
# runtime — no nvcc/CUDA toolkit needed on the host, only the driver
# (already present: nvidia-smi works outside any container here).
# All pins are installed last, after anything that might have upgraded
# them, same as the Dockerfile does.
micromamba run -n "${ENV_NAME}" pip install --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cu124 torch torchvision
micromamba run -n "${ENV_NAME}" pip install --no-cache-dir \
    ultralytics jupyterlab ipykernel matplotlib ipywidgets
micromamba run -n "${ENV_NAME}" pip install --no-cache-dir \
    'opencv-python<5' 'setuptools<80'

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

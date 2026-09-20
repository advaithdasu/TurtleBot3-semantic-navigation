#!/usr/bin/env bash
# Run this on the GPU HOST before the first `docker compose up`.
#
# Every failure it checks for is one whose downstream symptom is indirect:
# a missing container toolkit shows up as "the sim is mysteriously slow",
# a `compute,utility`-only driver capability as a black Gazebo camera, and
# an old driver as a torch CUDA error thrown deep inside YOLO. Catching
# them here costs 30 seconds instead of an afternoon.
#
#   ./docker/preflight.sh
set -uo pipefail

PASS=0
FAIL=0
WARN=0

ok()   { echo "  ✓ $*"; PASS=$((PASS + 1)); }
bad()  { echo "  ✗ $*"; FAIL=$((FAIL + 1)); }
warn() { echo "  ! $*"; WARN=$((WARN + 1)); }

echo "GPU host preflight"
echo

# ── 1. Host driver ────────────────────────────────────────────────────────
if command -v nvidia-smi >/dev/null 2>&1; then
  gpu_line="$(nvidia-smi --query-gpu=name,memory.total,driver_version \
                --format=csv,noheader 2>/dev/null | head -1)"
  if [ -n "${gpu_line}" ]; then
    ok "NVIDIA driver: ${gpu_line}"
    # torch cu124 wheels need a 525+ driver; the cu12 forward-compat story
    # is only for datacenter drivers, so check rather than assume.
    drv="${gpu_line##*, }"
    drv_major="${drv%%.*}"
    if [ "${drv_major:-0}" -lt 525 ] 2>/dev/null; then
      warn "driver ${drv} is older than 525 — the cu124 torch wheels in the sim"
      echo "      image may fail to initialise CUDA. Upgrade the driver, or pin an"
      echo "      older torch index URL in docker/Dockerfile."
    fi
    vram="$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1)"
    if [ -n "${vram}" ] && [ "${vram}" -lt 12000 ] 2>/dev/null; then
      warn "${vram} MiB VRAM — LocateAnything-3B wants ~12 GB. Run the grounding"
      echo "      sidecar with GROUNDING_BACKEND=mock, or expect an OOM at load."
    fi
  else
    bad "nvidia-smi ran but reported no GPU"
  fi
else
  bad "nvidia-smi not found — install the NVIDIA driver first"
fi

# ── 2. Docker ─────────────────────────────────────────────────────────────
if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
  ok "docker: $(docker --version | sed 's/,.*//')"
  if docker compose version >/dev/null 2>&1; then
    ok "compose: $(docker compose version --short 2>/dev/null)"
  else
    bad "\`docker compose\` (v2) not available — the v1 \`docker-compose\` binary"
    echo "      does not understand deploy.resources GPU reservations."
  fi
else
  bad "docker is not installed, not running, or this user cannot reach the daemon"
fi

# ── 3. Container toolkit: compute ─────────────────────────────────────────
# The single most common failure. Without it every container starts fine and
# simply has no GPU in it.
if docker info >/dev/null 2>&1; then
  if docker run --rm --gpus all ubuntu:24.04 nvidia-smi >/dev/null 2>&1; then
    ok "NVIDIA Container Toolkit: GPUs visible inside containers"
  else
    bad "containers cannot see the GPU. Install the NVIDIA Container Toolkit:"
    echo "      https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/"
    echo "      then: sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker"
  fi

  # ── 4. Container toolkit: graphics ──────────────────────────────────────
  # NVIDIA_DRIVER_CAPABILITIES=all is what makes libEGL_nvidia and its ICD
  # json get injected. With the default compute,utility they are absent,
  # gz-sim's ogre2 finds no EGL vendor and silently falls back to llvmpipe —
  # visible only as a real-time factor that will not go above ~0.3.
  if docker run --rm --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all ubuntu:24.04 \
       test -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json >/dev/null 2>&1; then
    ok "EGL vendor ICD injected (graphics capability works → GPU sensor rendering)"
  else
    warn "no NVIDIA EGL ICD inside the container. gz-sim will fall back to llvmpipe"
    echo "      software rendering. Usually means an old container toolkit; the"
    echo "      image already sets NVIDIA_DRIVER_CAPABILITIES=all."
  fi
fi

# ── 5. Ports ──────────────────────────────────────────────────────────────
# Published on 127.0.0.1, so a collision is with something else on this box.
for spec in "8888 JupyterLab" "${GUI_PORT:-6080} noVNC" "8801 grounding"; do
  port="${spec%% *}"; what="${spec#* }"
  if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | grep -q ":${port} "; then
    warn "port ${port} (${what}) is already in use — set GUI_PORT in docker/.env"
    echo "      or stop the other listener, otherwise \`compose up\` fails to bind."
  fi
done

# ── 6. Disk ───────────────────────────────────────────────────────────────
# sim image ~8 GB, grounding image ~6 GB, LocateAnything-3B weights ~7 GB.
avail_gb="$(df -BG --output=avail /var/lib/docker 2>/dev/null | tail -1 | tr -dc '0-9')"
if [ -n "${avail_gb}" ]; then
  if [ "${avail_gb}" -lt 40 ]; then
    warn "${avail_gb} GB free on /var/lib/docker — images + model weights need ~25 GB"
  else
    ok "disk: ${avail_gb} GB free for images and model weights"
  fi
fi

echo
echo "${PASS} ok, ${WARN} warning(s), ${FAIL} failure(s)"
if [ "${FAIL}" -gt 0 ]; then
  echo "Fix the ✗ items before building — none of them degrade gracefully."
  exit 1
fi
echo "Ready:  docker compose -f docker/compose.yaml up -d --build"

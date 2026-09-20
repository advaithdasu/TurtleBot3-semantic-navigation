#!/usr/bin/env bash
# Virtual desktop for the GUI tools (RViz, the Gazebo GUI) on a headless
# GPU host, reachable from a laptop through the same SSH tunnel as Jupyter.
#
#   docker compose -f docker/compose.yaml exec sim docker/start_gui.sh start
#   → http://localhost:6080/vnc.html   (after `ssh -L 6080:localhost:6080 ...`)
#
# Shape of it:
#
#   Xvfb :99 ──► fluxbox (window manager)
#      ▲
#      └── rviz2 / gz sim -g  (DISPLAY=:99, software GL)
#      │
#   x11vnc :5900 (container-local) ──► websockify/noVNC :6080 ──► browser
#
# What this is NOT: a rendering path for the simulator. gz-sim's Sensors
# system keeps rendering the robot's camera headless through EGL straight
# on the NVIDIA GPU (compose pins TB3_HEADLESS_RENDERING=true). Only the
# *viewer* processes draw on this X server, through Mesa llvmpipe, because
# there is no NVIDIA GLX behind Xvfb. That split is deliberate: pointing
# the sim server at this display instead would drop sensor rendering onto
# llvmpipe and collapse the real-time factor — the exact failure the EGL
# path exists to avoid.
#
# Expect the viewers to feel like a remote desktop, not like a local GPU:
# RViz is comfortable, the Gazebo 3D view is usable but not smooth. The
# notebook's inline map/camera views remain the low-latency option.
set -euo pipefail

CMD="${1:-start}"

DISPLAY_NUM="${TB3_GUI_DISPLAY_NUM:-99}"
GEOMETRY="${TB3_GUI_GEOMETRY:-1600x900x24}"
NOVNC_PORT="${TB3_GUI_PORT:-6080}"
VNC_PORT="${TB3_GUI_VNC_PORT:-5900}"

RUN_DIR="${TB3_GUI_RUN_DIR:-/tmp/tb3-gui}"
mkdir -p "${RUN_DIR}"

_pidfile() { echo "${RUN_DIR}/$1.pid"; }
_logfile() { echo "${RUN_DIR}/$1.log"; }

_alive() {
  local pf; pf="$(_pidfile "$1")"
  [ -f "${pf}" ] && kill -0 "$(cat "${pf}")" 2>/dev/null
}

# Start a background process, recording its pid, unless it is already up.
_spawn() {
  local name="$1"; shift
  if _alive "${name}"; then
    echo "  ${name}: already running (pid $(cat "$(_pidfile "${name}")"))"
    return 0
  fi
  "$@" >"$(_logfile "${name}")" 2>&1 &
  echo $! > "$(_pidfile "${name}")"
  echo "  ${name}: started (pid $!)  log: $(_logfile "${name}")"
}

_stop_one() {
  local name="$1" pf; pf="$(_pidfile "${name}")"
  if _alive "${name}"; then
    local pid; pid="$(cat "${pf}")"
    kill "${pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
    kill -9 "${pid}" 2>/dev/null || true
    echo "  ${name}: stopped"
  else
    echo "  ${name}: not running"
  fi
  rm -f "${pf}"
}

start() {
  export DISPLAY=":${DISPLAY_NUM}"

  echo "Starting virtual desktop on DISPLAY=${DISPLAY} (${GEOMETRY})"

  # -nolisten tcp: the X server is reachable only over its unix socket, so
  # nothing outside this container can open windows on it. x11vnc is the
  # only door in, and it binds to localhost.
  _spawn xvfb Xvfb ":${DISPLAY_NUM}" -screen 0 "${GEOMETRY}" -nolisten tcp -noreset

  # The launch files probe for this socket to decide whether the GUI can
  # come up (see _x_display_available in tb3_sim.launch.py), so wait for
  # it rather than racing a launch started seconds later.
  for _ in $(seq 1 40); do
    [ -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ] && break
    sleep 0.25
  done
  if [ ! -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ]; then
    echo "Xvfb did not come up; see $(_logfile xvfb)" >&2
    exit 1
  fi

  # Without a window manager, RViz and the Gazebo GUI come up as override
  # -redirect windows: no title bar, no way to move, resize or raise them,
  # and dialogs land stacked on top of each other.
  _spawn fluxbox fluxbox

  # Auth: a password if one was supplied, otherwise open — which is safe
  # only because the port is published on the host's 127.0.0.1 and reached
  # through SSH. Same posture as the grounding server on 8801.
  local auth_args=(-nopw)
  if [ -n "${TB3_VNC_PASSWORD:-}" ]; then
    x11vnc -storepasswd "${TB3_VNC_PASSWORD}" "${RUN_DIR}/vncpasswd" >/dev/null 2>&1
    auth_args=(-rfbauth "${RUN_DIR}/vncpasswd")
  fi

  # -localhost keeps x11vnc off the published interface; websockify, which
  # runs in this same container, is its only client.
  _spawn x11vnc x11vnc -display ":${DISPLAY_NUM}" -rfbport "${VNC_PORT}" \
      -localhost -forever -shared -noxdamage -repeat "${auth_args[@]}"

  # Ubuntu splits websockify across packages differently across releases:
  # sometimes /usr/bin/websockify, sometimes only the python module that
  # novnc pulls in. Take whichever is here rather than pinning one.
  local -a ws_cmd
  if command -v websockify >/dev/null 2>&1; then
    ws_cmd=(websockify)
  elif python3 -c "import websockify" >/dev/null 2>&1; then
    ws_cmd=(python3 -m websockify)
  else
    echo "no websockify found — is the novnc package installed?" >&2
    exit 1
  fi
  _spawn novnc "${ws_cmd[@]}" --web=/usr/share/novnc \
      "${NOVNC_PORT}" "localhost:${VNC_PORT}"

  sleep 1
  echo
  status || true
  cat <<MSG

Open it from your laptop (tunnel ${NOVNC_PORT} first — docker/tunnel.sh does this):

    http://localhost:${NOVNC_PORT}/vnc.html

Then launch with the viewers on:

    DISPLAY=:${DISPLAY_NUM} ros2 launch tb3_coordinator full_semantic_nav.launch.py \\
        use_rviz:=true use_gzclient:=true

(DISPLAY is already exported in this container; with the desktop up, both
default to true anyway. Sensor rendering stays on the GPU regardless.)
MSG
}

stop() {
  echo "Stopping virtual desktop"
  # Reverse order: viewers lose their server last.
  _stop_one novnc
  _stop_one x11vnc
  _stop_one fluxbox
  _stop_one xvfb
  rm -f "/tmp/.X${DISPLAY_NUM}-lock"
}

status() {
  local any=0
  echo "Virtual desktop (DISPLAY=:${DISPLAY_NUM}, noVNC ${NOVNC_PORT}):"
  for name in xvfb fluxbox x11vnc novnc; do
    if _alive "${name}"; then
      echo "  ✓ ${name} (pid $(cat "$(_pidfile "${name}")"))"
      any=1
    else
      echo "  ✗ ${name}"
    fi
  done
  [ -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ] \
    && echo "  ✓ X socket /tmp/.X11-unix/X${DISPLAY_NUM}" \
    || echo "  ✗ X socket /tmp/.X11-unix/X${DISPLAY_NUM} (launches will run headless)"
  return $(( any ? 0 : 1 ))
}

case "${CMD}" in
  start)   start ;;
  stop)    stop ;;
  restart) stop; start ;;
  status)  status ;;
  *) echo "usage: $0 {start|stop|restart|status}" >&2; exit 2 ;;
esac

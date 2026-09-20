#!/usr/bin/env bash
# Run this on your LAPTOP, not on the GPU host.
#
# Opens the three local forwards the stack needs and holds them:
#
#   8888 → JupyterLab            http://localhost:8888
#   6080 → noVNC desktop         http://localhost:6080/vnc.html
#   8801 → grounding server      http://localhost:8801/health
#
# Everything the containers publish is bound to the GPU host's 127.0.0.1
# (see docker/compose.yaml), so this tunnel is the only way in — which is
# the point: an open JupyterLab on a public IP is a root shell for whoever
# finds it, and the noVNC and grounding endpoints are unauthenticated.
#
#   ./docker/tunnel.sh advaith@gpu-host
#   ./docker/tunnel.sh -g 6081 advaith@gpu-host      # host-side GUI_PORT
#   ./docker/tunnel.sh -p 2222 advaith@gpu-host      # non-default ssh port
#
# Ctrl-C closes it. Add -f to background it instead.
set -euo pipefail

JUPYTER_PORT=8888
GUI_PORT=6080
GROUNDING_PORT=8801
SSH_PORT=""
BACKGROUND=0

usage() {
  # The header comment above is the help text; print it up to the first
  # line that is not a comment.
  awk 'NR > 1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "$0"
  exit "${1:-0}"
}

while getopts "j:g:s:p:fh" opt; do
  case "${opt}" in
    j) JUPYTER_PORT="${OPTARG}" ;;
    g) GUI_PORT="${OPTARG}" ;;
    s) GROUNDING_PORT="${OPTARG}" ;;
    p) SSH_PORT="${OPTARG}" ;;
    f) BACKGROUND=1 ;;
    h) usage 0 ;;
    *) usage 2 ;;
  esac
done
shift $((OPTIND - 1))

TARGET="${1:-}"
if [ -z "${TARGET}" ]; then
  echo "error: no ssh target given (e.g. user@gpu-host)" >&2
  usage 2
fi

# Refuse to start if something local already owns a port: ssh would print a
# single "bind: Address already in use" line and carry on with a tunnel that
# silently does nothing, which then looks like a broken container.
port_busy() { lsof -nP -iTCP:"$1" -sTCP:LISTEN >/dev/null 2>&1; }
for p in "${JUPYTER_PORT}" "${GUI_PORT}" "${GROUNDING_PORT}"; do
  if port_busy "${p}"; then
    echo "error: local port ${p} is already in use — close it, or pass a" >&2
    echo "       different local port (-j/-g/-s) and adjust the URLs." >&2
    exit 1
  fi
done

ssh_args=(-N
  -L "${JUPYTER_PORT}:localhost:8888"
  -L "${GUI_PORT}:localhost:${GUI_PORT}"
  -L "${GROUNDING_PORT}:localhost:8801"
  # Keep the tunnel from dying silently on an idle NAT/Wi-Fi link: a stack
  # you left running for an hour should still be reachable when you come back.
  -o ServerAliveInterval=30 -o ServerAliveCountMax=6 -o ExitOnForwardFailure=yes)
[ -n "${SSH_PORT}" ] && ssh_args+=(-p "${SSH_PORT}")
[ "${BACKGROUND}" -eq 1 ] && ssh_args+=(-f)

cat <<MSG
Tunnelling to ${TARGET}

  JupyterLab   http://localhost:${JUPYTER_PORT}       (token: JUPYTER_TOKEN, default 'tb3')
  noVNC GUI    http://localhost:${GUI_PORT}/vnc.html  (start it first:
                 docker compose -f docker/compose.yaml exec sim start_gui.sh start)
  grounding    http://localhost:${GROUNDING_PORT}/health

MSG
[ "${BACKGROUND}" -eq 0 ] && echo "Ctrl-C to close."
exec ssh "${ssh_args[@]}" "${TARGET}"

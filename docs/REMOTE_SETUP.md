# Remote GPU host + local Mac setup

This is the step-by-step version of [Running it](../README.md#running-it) in
the main README, organized by **which machine each command runs on**
instead of by topic. Use it when you want all the sim/GPU work happening on
a remote NVIDIA box while RViz, the Gazebo GUI, and JupyterLab show up in a
browser on your Mac.

Every port the stack publishes is bound to the GPU host's `127.0.0.1` only
(see `docker/compose.yaml`) — nothing is reachable directly from your Mac.
An SSH tunnel is not optional; it's the only way in.

## One-time setup — on the remote GPU host

`preflight.sh` (below) checks for Docker and the NVIDIA driver — it does not
install them. If `docker` isn't already on the box (Ubuntu/Debian):

```bash
# Docker Engine
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker "$USER"
newgrp docker            # or log out and back in

# NVIDIA Container Toolkit — needed for `docker run --gpus all` to work
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

The NVIDIA driver itself (`nvidia-smi`) is assumed to already be installed —
that's usually baked into the host image on cloud GPU providers; if it's
missing, install it separately before continuing.

Then clone the repo and run preflight:

```bash
git clone https://github.com/advaithdasu/TurtleBot3-semantic-navigation.git
cd TurtleBot3-semantic-navigation
./docker/preflight.sh
```

`preflight.sh` checks the NVIDIA driver, Docker Compose v2, GPU visibility
inside a container, the EGL vendor ICD, port collisions, and disk space —
run it before the first build so a missing piece shows up as a clear error
instead of a mysteriously slow sim or a black camera feed.

If something else on the host already owns port 6080 (check with
`ss -ltnp | grep 6080`), copy `docker/.env.example` to `docker/.env` and set
a different `GUI_PORT` there — the in-container port stays 6080 regardless.

## Every session — on the remote GPU host

```bash
cd TurtleBot3-semantic-navigation
docker compose -f docker/compose.yaml up -d --build
docker compose -f docker/compose.yaml logs -f sim     # watch for the JupyterLab URL, then Ctrl-C
```

This starts two containers: `sim` (ROS 2 + Gazebo + YOLOv8n, JupyterLab on
8888, noVNC on 6080) and `grounding` (LocateAnything-3B on 8801, sharing
`sim`'s network namespace). Leave them running — everything below just
connects to them.

## Every session — on your Mac

Open the tunnel (from the repo, or copy `docker/tunnel.sh` locally):

```bash
./docker/tunnel.sh user@gpu-host
```

This holds three forwards open (`8888` JupyterLab, `6080` noVNC, `8801`
grounding) with keepalives so an idle Wi-Fi link doesn't silently drop it.
If you set a non-default `GUI_PORT` on the host, pass the matching flag:
`./docker/tunnel.sh -g 6081 user@gpu-host`. Leave this terminal open for the
whole session — `Ctrl-C` closes the tunnel (or pass `-f` to background it).

In a browser on your Mac:

```
http://localhost:8888          # JupyterLab, token defaults to `tb3`
```

Open `notebooks/semantic_nav.ipynb` and run it top to bottom — it checks
the GPU, fetches the YOLOv8n weights, builds the workspace, and launches the
full stack, all executing on the remote host. The notebook also renders the
camera, SLAM map, and semantic landmarks inline, which is the lowest-latency
way to watch a run without the GUI at all.

## Optional: RViz / Gazebo GUI — on the remote GPU host

Start the virtual desktop **before** launching the ROS stack — the launch
files decide whether to enable the GUI by probing for the X socket once, at
launch time:

```bash
docker compose -f docker/compose.yaml exec -u ubuntu sim start_gui.sh start
```

(or `tb3_nb.gui_start()` from the notebook). `start_gui.sh status` reports
what's up; `start_gui.sh stop` tears it down.

## Optional: RViz / Gazebo GUI — on your Mac

With the tunnel from above still open:

```
http://localhost:6080/vnc.html
```

That's the real RViz and Gazebo GUI — TF trees, costmaps, Nav2 plans, free
camera movement — rendered in software (llvmpipe) inside the container and
streamed over noVNC. The simulator's own sensor rendering stays on the GPU
via EGL regardless (`TB3_HEADLESS_RENDERING=true`), so the viewers being
software-rendered doesn't cost you real-time factor — but the viewers
themselves do cost CPU, so turn the desktop off (`start_gui.sh stop`) when
you're measuring RTF. Set `TB3_VNC_PASSWORD` in `docker/.env` on the host if
you want the desktop password-protected; unset, it's open to anything that
can reach the port, which behind the tunnel is only you.

## Quick reference

| Step | Machine | Command |
| --- | --- | --- |
| Preflight (once) | remote | `./docker/preflight.sh` |
| Start containers | remote | `docker compose -f docker/compose.yaml up -d --build` |
| Open tunnel | Mac | `./docker/tunnel.sh user@gpu-host` |
| Drive the sim | Mac (browser) | `http://localhost:8888` → `notebooks/semantic_nav.ipynb` |
| Start GUI desktop | remote | `docker compose -f docker/compose.yaml exec -u ubuntu sim start_gui.sh start` |
| View GUI | Mac (browser) | `http://localhost:6080/vnc.html` |
| Stop containers | remote | `docker compose -f docker/compose.yaml down` |

See the main [README](../README.md#running-it) for world selection, sending
semantic commands, and the LocateAnything grounding queries — all of that
happens from the notebook or a shell inside the `sim` container once this
setup is in place.

# Native setup — no Docker, no root (shared GPU cluster / JupyterHub)

Use this instead of [`REMOTE_SETUP.md`](REMOTE_SETUP.md) when the "remote GPU
host" is actually a **shared, multi-user GPU cluster node reached through
JupyterHub**, where:

- `sudo` fails with `"no new privileges" flag is set` — a systemd hardening
  flag on your session (SystemdSpawner-style JupyterHub), not a config bug.
- There is no `docker`, `podman`, `apptainer`, or `singularity` on the box —
  no container runtime at all, so `docker compose` cannot run here.
- `nvidia-smi` **does** work directly — the GPU itself is reachable, it's
  specifically containers that aren't available.

If that's not your situation — if you actually have a root-owned Linux VM
or bare-metal box — use [`REMOTE_SETUP.md`](REMOTE_SETUP.md) with Docker
instead; it's simpler and closer to how the project is normally run and
tested.

## How this differs from the Docker path

The Docker image installs ROS 2 Jazzy + Gazebo Harmonic via `apt` inside the
container. With no root, `apt` isn't available either. The substitute is
[RoboStack](https://robostack.github.io/), which publishes the same ROS 2
Jazzy + Gazebo Harmonic packages as **prebuilt conda packages** — no
compilation, no root, everything lands under `~/micromamba`. Every ROS
package this workspace's `package.xml` files depend on (`navigation2`,
`slam_toolbox`, `cv_bridge`, `vision_msgs`, `tf2_geometry_msgs`,
`turtlebot3_gazebo`, `ros_gz_sim`/`bridge`/`image`, `ament_lint_auto`, ...)
is confirmed present on the `robostack-jazzy` conda channel.

torch, ultralytics (YOLOv8n), and JupyterLab are installed with `pip` into
the same conda env, exactly like the Docker image does inside the
container — same `cu124` wheel index (vendors its own CUDA runtime, so no
`nvcc`/CUDA toolkit needed — only the driver, which is already on this
box), and the same `opencv-python<5` pin the Dockerfile carries, for the
same reason: an unpinned `opencv-python` resolves to 5.x, which renumbers
`CV_8UC*` constants relative to what `cv_bridge`'s compiled extension
expects, and throws `KeyError(16)` at conversion time.

**Not yet covered here: the RViz/Gazebo GUI over a browser.** The Docker
path's noVNC chain (`Xvfb` + `x11vnc` + `fluxbox` + `websockify` + `noVNC`)
needs two apt-only pieces (`x11vnc`, `fluxbox`) that aren't on conda-forge.
[`xpra`](https://www.xpra.org/) *is* on conda-forge and is built for exactly
this (screen/app forwarding with an HTML5 client, no root), but it isn't
scripted here yet — ask if you want it built out. Until then, drive the
stack from the notebook's inline camera/map/landmark views, which need no
display at all (see below).

## 1. One-time environment setup

```bash
cd TurtleBot3-semantic-navigation   # wherever you cloned it, e.g. /data/users/<you>/...
./native/install_env.sh
```

This installs `micromamba` (a single self-contained binary, no root) into
`~/micromamba` if it isn't already on your `PATH`, then creates a `tb3`
conda env with ROS 2 Jazzy + Gazebo Harmonic + the rest of the ROS
dependencies, installs torch/ultralytics/JupyterLab with pip, and registers
a Jupyter kernel ("TB3 Semantic Nav (RoboStack)") that your existing
JupyterHub session will pick up — refresh the JupyterLab launcher tab (no
server restart needed) and it should appear as a kernel option.

Takes a while the first time (conda solve + package downloads). Re-running
it is safe — `micromamba create -y` overwrites the env in place.

## 2. Build the workspace

```bash
micromamba activate tb3
./native/setup_ws.sh
```

Downloads the YOLOv8n weights (git-ignored, ~6 MB) if missing, then runs
`colcon build` against the **active conda env's** Python — deliberately
different from the repo's existing `build.sh`, which forces `/usr/bin/python3`
to dodge a conda install it assumes is a stray on an apt-based system. Here
conda's Python is the correct one: it's what `ros-jazzy-cv-bridge` and
`colcon-common-extensions` were built against. Don't use `build.sh` on this
path.

## 3. Run it

From a JupyterLab terminal or notebook, every session:

```bash
micromamba activate tb3
source install/setup.bash
```

(the two `source` lines from `docker/entrypoint.sh`, translated — RoboStack's
conda activation already puts `ros2`/`colcon`/`rclpy` on the path the moment
you `activate tb3`, so there's no `/opt/ros/jazzy/setup.bash` to source.)

Then either:

- **From a terminal:**
  ```bash
  ros2 launch tb3_coordinator full_semantic_nav.launch.py
  ```
  With no `DISPLAY` set (the normal state of a JupyterHub session), the
  launch files' own probe (`os.path.exists("/tmp/.X11-unix/X...")`) finds no
  X server and defaults `use_rviz`/`use_gzclient` to **false** and
  `headless_rendering` to **true** automatically — no env vars to set. The
  camera still renders through EGL straight on the GPU; only the viewer
  windows are skipped, same split as the Docker path
  (`TB3_HEADLESS_RENDERING=true`), just arrived at as the default instead of
  an explicit override.

- **From the notebook:** select the "TB3 Semantic Nav (RoboStack)" kernel
  and work through `notebooks/semantic_nav.ipynb` as described in the main
  [README](../README.md#2-drive-it-from-the-notebook) — the inline
  camera/map/landmark rendering needs no display and is the natural fit
  here, GUI or not.

## What's unverified

This path has not been run end-to-end on real hardware yet — only checked
against the RoboStack package index (every needed `ros-jazzy-*` package
confirmed present) and the launch files' existing headless-default logic.
Specifically untested:

- Whether conda's `libglvnd`/EGL dispatch libraries successfully hand off to
  the host's NVIDIA driver's EGL implementation with no container runtime
  in the loop. On bare metal this *should* just work — the NVIDIA driver
  installs its EGL vendor ICD JSON directly onto the host filesystem
  (that's what the NVIDIA Container Toolkit injects into a container; here
  there's no container, so it's already where `libglvnd` looks) — but it
  hasn't been watched happen. If gz-sim silently falls back to `llvmpipe`,
  the symptom is the same as the Docker image's warning: a real-time factor
  sitting around 0.3 instead of near 1.0, and `nvidia-smi` showing no
  `gz sim` process holding VRAM while a stack is running.
- Whether `colcon build` in a RoboStack env behaves exactly like the
  Docker/apt one for this workspace's mixed `ament_python`/`ament_cmake`
  packages (`tb3_query` generates a message via `rosidl_default_generators`,
  `tb3_frontier_exploration` pulls in `ament_lint_auto` under
  `BUILD_TESTING=ON`) — first real run hit one issue, now fixed: pip
  installs in `install_env.sh` were letting `setuptools` drift past 80.0.0,
  which drops the legacy `develop --editable` flag colcon's
  `--symlink-install` needs for `ament_python` packages and fails with
  `error: option --editable not recognized`
  ([pypa/setuptools#4971](https://github.com/pypa/setuptools/issues/4971),
  hits ROS 2 Jazzy specifically per
  [ros2/ros2#1702](https://github.com/ros2/ros2/issues/1702)).
  `install_env.sh` now pins `setuptools<80`, same as `docker/Dockerfile`
  already does for the same reason. If you set up your env before this fix,
  run `pip install 'setuptools<80'` inside it and re-run `setup_ws.sh`.
  Still open: whether the rest of the build (message generation, the
  `ament_cmake` packages) goes cleanly past that point.
- The 11 GB VRAM per GPU on this box vs. LocateAnything-3B's ~12 GB ask —
  run the grounding sidecar path with the mock backend
  (`GROUNDING_BACKEND=mock`, or set `grounding_server_url` in
  [`semantic_query.yaml`](../src/tb3_query/config/semantic_query.yaml) to
  point at a server elsewhere) unless you've confirmed it fits.

Report back what actually happens at each step — this doc should get
tightened up into "known good" once it's been run for real.

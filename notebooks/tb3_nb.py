"""Helpers backing semantic_nav.ipynb — run the semantic-nav stack from a
notebook on the GPU host.

Everything the notebook needs is here so the cells stay short enough to
read at a glance. Three pieces:

  * ``check_env()`` / ``build_workspace()`` — one-time setup, wrapping the
    same ``./build.sh`` the shell workflow uses.
  * ``Stack`` — owns the ``ros2 launch`` process. Notebook kernels outlive
    individual cells, so the launch runs detached with its output going to
    a log file that later cells tail; nothing blocks the kernel.
  * ``Ros`` — one long-lived rclpy node spinning on a background thread,
    caching the latest message per topic. Cells then read state
    synchronously (``ros.map()``, ``ros.status()``) instead of each one
    having to build and spin its own node.

Why this file exists rather than raw cells: rclpy in a notebook has two
sharp edges — double ``rclpy.init()`` on cell re-run, and executors that
never get spun — and both are handled once here instead of in every
notebook that touches the stack.

Imports are deliberately lazy in places (matplotlib, tf2) so that
importing this module inside a plain ROS shell, or on a machine without a
display stack, does not fail.
"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
# notebooks/ lives at the workspace root, so the repo is one level up.
WS_ROOT = Path(__file__).resolve().parent.parent
LOG_DIR = WS_ROOT / "log" / "notebook"


# ---------------------------------------------------------------------------
# Topic map — single source of truth for the notebook
# ---------------------------------------------------------------------------
# Keep in sync with the config files noted alongside each entry. These are
# the resolved (absolute) names, i.e. what `ros2 topic list` prints, not
# the `~/...` relative forms used in the yaml.
TOPICS = {
    "camera":      "/camera/image_raw",                              # tb3_sim bridge
    "debug_image": "/detector_node/debug_image",                     # tb3_detector/config/detector.yaml
    "detections":  "/detector_node/detections",
    "map":         "/map",                                           # slam_toolbox via nav2_bringup
    "landmarks":   "/semantic_map_memory_node/landmark_objects",     # tb3_coordinator/config/coordinator.yaml
    "status":      "/coordinator_node/status",                       # tb3_coordinator/config/coordinator.yaml
    "query_status": "/semantic_query_node/query_status",             # tb3_query/config/semantic_query.yaml
    "user_command": "/user_command",
}


# ---------------------------------------------------------------------------
# Environment / build
# ---------------------------------------------------------------------------
def _run(cmd, **kw):
    """Run a command, returning (rc, combined output). Never raises."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=60, **kw)
        return p.returncode, (p.stdout + p.stderr).strip()
    except FileNotFoundError:
        return 127, f"{cmd[0]}: not found"
    except subprocess.TimeoutExpired:
        return 124, f"{cmd[0]}: timed out"


def check_env(verbose: bool = True) -> dict:
    """Report whether this container can actually do what the stack needs.

    Checks the things that fail *late and confusingly* if wrong:
    a GPU that is invisible to the container shows up only as a mysteriously
    slow sim, and a torch CPU build only as YOLO quietly running at 2 Hz.
    """
    info: dict = {}

    rc, out = _run(["nvidia-smi",
                    "--query-gpu=name,memory.total,driver_version",
                    "--format=csv,noheader"])
    info["gpu"] = out if rc == 0 else None

    try:
        import torch
        info["torch"] = torch.__version__
        info["torch_cuda_build"] = torch.version.cuda
        info["cuda_available"] = torch.cuda.is_available()
        info["cuda_device"] = (torch.cuda.get_device_name(0)
                               if torch.cuda.is_available() else None)
    except Exception as e:                                  # noqa: BLE001
        info["torch"] = f"import failed: {e}"
        info["cuda_available"] = False

    info["ros_distro"] = os.environ.get("ROS_DISTRO")
    info["tb3_model"] = os.environ.get("TURTLEBOT3_MODEL")
    info["display"] = os.environ.get("DISPLAY")  # expected unset → EGL path

    rc, out = _run(["gz", "sim", "--versions"])
    info["gz_sim"] = out.splitlines()[0] if rc == 0 and out else None

    info["workspace_built"] = (WS_ROOT / "install" / "setup.bash").is_file()
    info["yolo_weights"] = (WS_ROOT / "src" / "tb3_detector" / "models"
                            / "yolov8n.pt").is_file()

    if verbose:
        ok = "✓"
        bad = "✗"
        def line(label, value, good):
            print(f"  {ok if good else bad} {label:<18} {value}")

        print("Environment")
        line("GPU", info["gpu"] or "no nvidia-smi", bool(info["gpu"]))
        line("torch", f"{info.get('torch')} (cuda build {info.get('torch_cuda_build')})",
             bool(info.get("torch_cuda_build")))
        line("torch.cuda", info.get("cuda_device") or "NOT available",
             info.get("cuda_available", False))
        line("ROS", info["ros_distro"] or "not sourced", bool(info["ros_distro"]))
        line("Gazebo", info["gz_sim"] or "gz not found", bool(info["gz_sim"]))
        line("DISPLAY", info["display"] or "unset (headless EGL rendering)", True)
        line("workspace built", info["workspace_built"], info["workspace_built"])
        line("yolov8n.pt", info["yolo_weights"], info["yolo_weights"])

        if not info.get("cuda_available"):
            print("\n  ! torch cannot see a GPU. The stack will still run, but "
                  "YOLO falls back to CPU.\n"
                  "    Check the container was started with the NVIDIA runtime "
                  "(deploy.resources in docker/compose.yaml).")

    return info


def fetch_weights() -> Path:
    """Download yolov8n.pt if missing. The weights are git-ignored."""
    dest = WS_ROOT / "src" / "tb3_detector" / "models" / "yolov8n.pt"
    if dest.is_file():
        print(f"weights already present: {dest}")
        return dest
    url = ("https://github.com/ultralytics/assets/releases/download/"
           "v8.2.0/yolov8n.pt")
    print(f"downloading {url}")
    dest.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["curl", "-fL", "-o", str(dest), url], check=True)
    print(f"saved to {dest}")
    return dest


def build_workspace(packages: list[str] | None = None) -> int:
    """Run ./build.sh, streaming output into the cell.

    Streams line by line rather than capturing: a colcon build takes
    minutes and a silent cell is indistinguishable from a hung one.
    """
    cmd = [str(WS_ROOT / "build.sh")]
    if packages:
        cmd += ["--packages-select", *packages]
    print("$", " ".join(cmd), flush=True)
    proc = subprocess.Popen(cmd, cwd=WS_ROOT, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, bufsize=1)
    for line in proc.stdout:                                # type: ignore[union-attr]
        print(line, end="", flush=True)
    rc = proc.wait()
    print(f"\n[build.sh exited {rc}]")
    if rc == 0:
        print("Restart the notebook kernel so the new install/ is on the "
              "Python path before importing any workspace message types.")
    return rc


# ---------------------------------------------------------------------------
# Launch process management
# ---------------------------------------------------------------------------
class Stack:
    """The ``ros2 launch`` process for the full semantic nav stack.

    Runs detached in its own process group with output to a log file.
    Both details matter:

      * detached (``start_new_session``) so ``stop()`` can signal the whole
        tree — ``ros2 launch`` forks a dozen nodes and killing only the
        parent leaves gz-sim, Nav2 and SLAM running, which then fight the
        next launch over topic names and the sim clock.
      * to a file rather than a pipe, because a notebook cell that finishes
        stops draining the pipe; once the OS buffer fills, every node in
        the stack blocks on its next log write and the sim silently stalls.
    """

    def __init__(self, world: str = "warehouse_models_person",
                 launch_args: list[str] | None = None,
                 log_path: str | os.PathLike | None = None) -> None:
        self.world = world
        self.launch_args = launch_args or []
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.log_path = Path(log_path) if log_path else LOG_DIR / "stack.log"
        self._proc: subprocess.Popen | None = None
        self._log_fh = None
        self._read_pos = 0

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> "Stack":
        if self.is_running:
            print("stack already running (pid "
                  f"{self._proc.pid}); call stop() first")   # type: ignore[union-attr]
            return self

        cmd = ["ros2", "launch", "tb3_coordinator", "full_semantic_nav.launch.py",
               f"world:={self.world}", *self.launch_args]
        self._log_fh = open(self.log_path, "w")
        self._read_pos = 0
        print("$", " ".join(cmd))
        print(f"  log: {self.log_path}")
        self._proc = subprocess.Popen(
            cmd, cwd=WS_ROOT, stdout=self._log_fh, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print(f"  pid: {self._proc.pid}  (startup ladder takes ~60 s "
              "before the coordinator is up)")
        return self

    @property
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def stop(self, timeout: float = 20.0) -> None:
        """SIGINT the process group, escalating to SIGKILL.

        SIGINT first because that is the signal ros2 launch treats as an
        orderly shutdown request; SIGTERM makes it leave orphans behind.
        """
        if self._proc is None:
            print("no stack process")
            return
        if self._proc.poll() is not None:
            print(f"stack already exited (rc={self._proc.returncode})")
            self._close_log()
            return

        pgid = os.getpgid(self._proc.pid)
        print(f"SIGINT -> process group {pgid} ...")
        os.killpg(pgid, signal.SIGINT)
        deadline = time.time() + timeout
        while time.time() < deadline and self._proc.poll() is None:
            time.sleep(0.5)

        if self._proc.poll() is None:
            print("still alive; SIGKILL")
            os.killpg(pgid, signal.SIGKILL)
            self._proc.wait(timeout=10)

        print(f"stack stopped (rc={self._proc.returncode})")
        self._close_log()

    def _close_log(self) -> None:
        if self._log_fh:
            self._log_fh.close()
            self._log_fh = None

    # -- log access --------------------------------------------------------
    def logs(self, n: int = 40, grep: str | None = None) -> None:
        """Print the last n log lines, optionally filtered."""
        if not self.log_path.is_file():
            print("no log file yet")
            return
        lines = self.log_path.read_text(errors="replace").splitlines()
        if grep:
            lines = [ln for ln in lines if grep.lower() in ln.lower()]
        for ln in lines[-n:]:
            print(ln)

    def follow(self, seconds: float = 30.0) -> None:
        """Stream new log lines for a while, then return control to the cell."""
        deadline = time.time() + seconds
        with open(self.log_path, "r", errors="replace") as fh:
            fh.seek(self._read_pos)
            while time.time() < deadline:
                chunk = fh.read()
                if chunk:
                    print(chunk, end="", flush=True)
                else:
                    time.sleep(0.4)
            self._read_pos = fh.tell()


# ---------------------------------------------------------------------------
# ROS session
# ---------------------------------------------------------------------------
class Ros:
    """A single rclpy node spinning on a background thread.

    Subscriptions are created on demand and only the newest message per
    topic is kept — the notebook always wants "what is true now", and an
    unbounded backlog of 30 Hz camera frames in kernel memory is a leak.
    """

    def __init__(self, name: str = "tb3_notebook") -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.executors import SingleThreadedExecutor

        # A notebook re-runs cells; rclpy.init() twice raises. Guard rather
        # than making the user restart the kernel.
        if not rclpy.ok():
            rclpy.init()

        # use_sim_time is required for TF: every transform in the stack is
        # stamped with the Gazebo clock, so a wall-clock listener finds
        # every lookup either far-future or long-expired.
        self._node = Node(name, parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)

        self._latest: dict[str, object] = {}
        self._subs: dict[str, object] = {}
        self._pubs: dict[str, object] = {}
        self._lock = threading.Lock()

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

        self._tf_buffer = None
        self._tf_listener = None

    def _spin(self) -> None:
        while not self._stop.is_set():
            self._exec.spin_once(timeout_sec=0.1)

    def shutdown(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3)
        self._node.destroy_node()

    # -- QoS ---------------------------------------------------------------
    @staticmethod
    def _qos(kind: str):
        """QoS profiles matched to each publisher.

        Mismatches here fail silently — the subscription is created, no
        error is raised, and no message ever arrives. /map in particular
        is TRANSIENT_LOCAL: subscribing VOLATILE means you see nothing
        until SLAM next republishes the whole grid.
        """
        from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                               DurabilityPolicy, HistoryPolicy)
        if kind == "sensor":       # camera, debug_image
            return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
        if kind == "map":          # latched occupancy grid
            return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
        return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)

    # -- subscriptions -----------------------------------------------------
    def watch(self, topic: str, msg_type, qos_kind: str = "default") -> None:
        """Start caching the newest message on `topic`."""
        if topic in self._subs:
            return

        def cb(msg, _t=topic):
            with self._lock:
                self._latest[_t] = msg

        self._subs[topic] = self._node.create_subscription(
            msg_type, topic, cb, self._qos(qos_kind))

    def latest(self, topic: str):
        with self._lock:
            return self._latest.get(topic)

    def wait_for(self, topic: str, timeout: float = 30.0):
        """Block until a message has arrived on `topic` (or timeout)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.latest(topic)
            if msg is not None:
                return msg
            time.sleep(0.2)
        return None

    def topics(self) -> list[str]:
        return sorted(n for n, _ in self._node.get_topic_names_and_types())

    # -- standard subscriptions -------------------------------------------
    def watch_all(self) -> None:
        """Subscribe to everything the notebook visualises."""
        from sensor_msgs.msg import Image
        from nav_msgs.msg import OccupancyGrid
        from std_msgs.msg import String
        from vision_msgs.msg import Detection2DArray, Detection3DArray

        self.watch(TOPICS["camera"], Image, "sensor")
        self.watch(TOPICS["debug_image"], Image, "sensor")
        self.watch(TOPICS["detections"], Detection2DArray)
        self.watch(TOPICS["map"], OccupancyGrid, "map")
        self.watch(TOPICS["landmarks"], Detection3DArray)
        self.watch(TOPICS["status"], String)
        self.watch(TOPICS["query_status"], String)

    # -- commands ----------------------------------------------------------
    def send_command(self, text: str, wait_for_sub: float = 5.0) -> bool:
        """Publish a semantic command on /user_command.

        Waits for the coordinator's subscription to be matched first. A
        publisher created and used in the same breath usually loses the
        first message: discovery has not completed, so the sample is
        published into a void and the robot simply never reacts.
        """
        from std_msgs.msg import String

        topic = TOPICS["user_command"]
        pub = self._pubs.get(topic)
        if pub is None:
            pub = self._node.create_publisher(String, topic, self._qos("default"))
            self._pubs[topic] = pub

        deadline = time.time() + wait_for_sub
        while time.time() < deadline and pub.get_subscription_count() == 0:
            time.sleep(0.1)
        if pub.get_subscription_count() == 0:
            print(f"! nothing is subscribed to {topic} — is the coordinator up? "
                  "(it starts ~55 s after launch)")
            return False

        pub.publish(String(data=text))
        print(f"→ {topic}: {text!r}")
        return True

    # -- state readers -----------------------------------------------------
    def status(self) -> str | None:
        msg = self.latest(TOPICS["status"])
        return msg.data if msg is not None else None

    def query_status(self) -> str | None:
        msg = self.latest(TOPICS["query_status"])
        return msg.data if msg is not None else None

    def landmarks(self) -> list[dict]:
        """Persistent semantic landmarks as plain dicts, map frame."""
        msg = self.latest(TOPICS["landmarks"])
        if msg is None:
            return []
        out = []
        for det in msg.detections:
            pos = det.bbox.center.position
            label = det.results[0].hypothesis.class_id if det.results else "?"
            score = det.results[0].hypothesis.score if det.results else 0.0
            out.append({"id": det.id or label, "label": label,
                        "x": pos.x, "y": pos.y, "z": pos.z, "score": score})
        return sorted(out, key=lambda d: d["id"])

    def detections(self) -> list[dict]:
        """Current 2-D YOLO detections."""
        msg = self.latest(TOPICS["detections"])
        if msg is None:
            return []
        return [{"label": d.results[0].hypothesis.class_id,
                 "conf": round(d.results[0].hypothesis.score, 3),
                 "cx": round(d.bbox.center.position.x, 1),
                 "cy": round(d.bbox.center.position.y, 1)}
                for d in msg.detections if d.results]

    def robot_pose(self) -> tuple[float, float, float] | None:
        """(x, y, yaw) of base_link in the map frame, or None if TF is not
        ready. Created lazily — the listener is only worth its CPU once
        something asks for a pose."""
        import math
        from rclpy.duration import Duration

        if self._tf_buffer is None:
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self._node,
                                                  spin_thread=False)
            time.sleep(1.0)  # let the buffer fill before the first lookup

        try:
            from rclpy.time import Time
            tf = self._tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=1.0))
        except Exception:                                    # noqa: BLE001
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        return (t.x, t.y, yaw)

    # -- readiness ---------------------------------------------------------
    def wait_until_ready(self, timeout: float = 180.0, poll: float = 5.0) -> bool:
        """Block until the stack is actually usable, reporting as it goes.

        "Usable" means SLAM is publishing a map, the detector is alive, and
        the coordinator has announced a state — i.e. all three phases of
        the launch ladder have landed, not merely that ros2 launch is
        running.
        """
        self.watch_all()
        checks = {
            "map (SLAM)":            lambda: self.latest(TOPICS["map"]) is not None,
            "camera frames":         lambda: self.latest(TOPICS["camera"]) is not None,
            "detector":              lambda: TOPICS["detections"] in self.topics(),
            "coordinator status":    lambda: self.status() is not None,
        }
        deadline = time.time() + timeout
        done: set[str] = set()
        while time.time() < deadline:
            for name, fn in checks.items():
                if name not in done and fn():
                    done.add(name)
                    print(f"  ✓ {name}  (+{int(time.time() - deadline + timeout)}s)")
            if len(done) == len(checks):
                print("stack ready.")
                return True
            time.sleep(poll)

        missing = [n for n in checks if n not in done]
        print(f"! timed out after {timeout:.0f}s; still missing: {', '.join(missing)}")
        print("  Inspect the launch log: stack.logs(60)")
        return False


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def image_to_array(msg) -> np.ndarray:
    """sensor_msgs/Image → RGB uint8 array.

    Decoded by hand rather than through cv_bridge on purpose. cv_bridge's
    compiled extension carries the OpenCV-4 CV_* type constants, and pip
    opencv >= 4.12 renumbered them, so conversion raises KeyError(16) —
    the same bug detector_node works around on the publish side. This
    reads the buffer directly and cannot hit it.
    """
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ("bgr8", "rgb8"):
        arr = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        return arr[:, :, ::-1] if enc == "bgr8" else arr
    if enc in ("mono8", "8uc1"):
        arr = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return np.dstack([arr] * 3)
    if enc in ("bgra8", "rgba8"):
        arr = buf.reshape(msg.height, msg.step // 4, 4)[:, :msg.width, :3]
        return arr[:, :, ::-1] if enc == "bgra8" else arr
    raise ValueError(f"unsupported image encoding {msg.encoding!r}")


def show_image(msg, title: str = "", figsize=(7, 5)) -> None:
    import matplotlib.pyplot as plt
    if msg is None:
        print(f"no image yet{(' for ' + title) if title else ''}")
        return
    plt.figure(figsize=figsize)
    plt.imshow(image_to_array(msg))
    plt.title(title or "camera")
    plt.axis("off")
    plt.show()


def show_camera(ros: "Ros", debug: bool = True, figsize=(7, 5)) -> None:
    """Show the detector's annotated frame (or the raw camera).

    Note detector_node only publishes ~/debug_image while something is
    subscribed to it, so the first call after `watch_all()` may find
    nothing — give it a second and retry.
    """
    topic = TOPICS["debug_image"] if debug else TOPICS["camera"]
    msg = ros.latest(topic)
    if msg is None and debug:
        msg = ros.wait_for(topic, timeout=5.0)
    show_image(msg, title=topic, figsize=figsize)


def show_map(ros: "Ros", figsize=(8, 8), show_robot: bool = True) -> None:
    """Plot the SLAM occupancy grid with semantic landmarks on top.

    This is the notebook's replacement for the RViz view: same three
    layers (map, landmark markers, robot pose), rendered inline.
    """
    import matplotlib.pyplot as plt

    grid = ros.latest(TOPICS["map"])
    if grid is None:
        print("no /map yet — SLAM may still be starting")
        return

    info = grid.info
    data = np.array(grid.data, dtype=np.int8).reshape(info.height, info.width)

    # -1 unknown, 0 free, 100 occupied → grey / white / black.
    img = np.full(data.shape, 0.6)          # unknown
    img[data == 0] = 1.0                    # free
    img[data > 50] = 0.0                    # occupied

    ox, oy = info.origin.position.x, info.origin.position.y
    res = info.resolution
    extent = (ox, ox + info.width * res, oy, oy + info.height * res)

    plt.figure(figsize=figsize)
    plt.imshow(img, cmap="gray", origin="lower", extent=extent,
               vmin=0.0, vmax=1.0, interpolation="nearest")

    for lm in ros.landmarks():
        plt.plot(lm["x"], lm["y"], "o", color="tab:red", markersize=9,
                 markeredgecolor="white")
        plt.annotate(lm["id"], (lm["x"], lm["y"]),
                     textcoords="offset points", xytext=(8, 5),
                     fontsize=9, color="tab:red",
                     bbox=dict(boxstyle="round,pad=0.15", fc="white",
                               ec="none", alpha=0.75))

    if show_robot:
        pose = ros.robot_pose()
        if pose is not None:
            x, y, yaw = pose
            plt.plot(x, y, "o", color="tab:blue", markersize=10,
                     markeredgecolor="white")
            plt.arrow(x, y, 0.35 * np.cos(yaw), 0.35 * np.sin(yaw),
                      head_width=0.12, color="tab:blue", length_includes_head=True)

    plt.title(f"SLAM map  ·  {len(ros.landmarks())} landmarks  ·  "
              f"state={ros.status() or '?'}")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid(alpha=0.15)
    plt.show()


def watch_navigation(ros: "Ros", seconds: float = 90.0, interval: float = 3.0) -> None:
    """Print coordinator state transitions until navigation settles.

    Terminal states come straight from the coordinator's state machine
    (EXPLORING → SEMANTIC_QUERYING → SEMANTIC_NAV → TARGET_REACHED/FAILED).
    """
    deadline = time.time() + seconds
    last = None
    while time.time() < deadline:
        state = ros.status()
        if state != last:
            pose = ros.robot_pose()
            where = f"  @({pose[0]:+.2f}, {pose[1]:+.2f})" if pose else ""
            print(f"[{time.strftime('%H:%M:%S')}] {state}{where}")
            last = state
        if state and any(k in state.upper()
                         for k in ("TARGET_REACHED", "FAILED", "ABORTED")):
            print("navigation finished.")
            return
        time.sleep(interval)
    print(f"still {last!r} after {seconds:.0f}s")


# ---------------------------------------------------------------------------
# Grounding server
# ---------------------------------------------------------------------------
def grounding_health(url: str = "http://127.0.0.1:8801") -> dict | None:
    """Ask the grounding sidecar what backend it loaded.

    Worth checking before any attribute query: with the mock backend the
    pipeline still answers, but only for colour words, so a failed
    "sofa with warm color" means something different in each case.
    """
    import json
    import urllib.request
    try:
        with urllib.request.urlopen(f"{url}/health", timeout=5) as r:
            info = json.load(r)
        print(f"grounding server: backend={info.get('backend')} "
              f"device={info.get('device')} model={info.get('model')}")
        if info.get("backend") == "mock":
            print("  ! mock backend — colour attributes only. Set "
                  "GROUNDING_BACKEND=locate_anything in compose to use the model.")
        return info
    except Exception as e:                                   # noqa: BLE001
        print(f"grounding server unreachable at {url}: {e}")
        print("  The query node degrades to nearest-first selection without it.")
        return None

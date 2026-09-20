"""
tb3_sim.launch.py — Gazebo Harmonic (gz-sim) + TurtleBot3 bringup.

Single entry point for "start the simulator with a robot in it", shared by
detector_test_sim, warehouse_semantic_sim and full_semantic_nav.

Composes:
  1. gz-sim server (``-r -s``) on the requested world, rendering through
     EGL when headless_rendering:=true, plus the GUI client (``-g``)
     unless use_gzclient:=false
  2. robot_state_publisher, from turtlebot3_gazebo's URDF
  3. the waffle_pi SDF, spawned via ``ros_gz_sim create``
  4. ros_gz_bridge + ros_gz_image for clock / odom / tf / scan / imu /
     joint_states / cmd_vel / camera

We deliberately do NOT include turtlebot3_gazebo's own
spawn_turtlebot3.launch.py: its bridge config maps /cmd_vel as
geometry_msgs/TwistStamped, while this workspace and Nav2 publish plain
Twist. config/gz_bridge.yaml is our replacement — see the header there.

Usage (normally included, not run directly):
    ros2 launch tb3_frontier_exploration tb3_sim.launch.py \
        world:=/abs/path/to/some.world x_pose:=-1.5 y_pose:=0.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# On the GPU host there is no X server: gz-sim's Sensors system renders
# camera frames through OGRE, which needs *some* display unless it is told
# to use EGL instead. `--headless-rendering` is that switch — ogre2 then
# creates an EGL context directly on the NVIDIA device, which is both the
# only way this works without X and considerably faster than the llvmpipe
# path the macOS image used.
#
# Deriving the defaults from the display rather than hardcoding them keeps
# a workstation with a real display (or X forwarding) working unchanged:
# there, gz renders to X and the GUI/RViz come up as before. In the
# container the same two questions have different answers — see the
# helpers below.
def _x_display_available() -> bool:
    """True when DISPLAY names an X server that is actually accepting clients.

    Deliberately not "is DISPLAY set". On the GPU host compose exports
    DISPLAY=:99 unconditionally, because that is where docker/start_gui.sh
    puts the Xvfb desktop it serves over noVNC — but that desktop may not
    be running. Launching RViz or the Gazebo GUI against a display with no
    server behind it aborts in Qt ("cannot connect to X server :99") and
    takes the whole launch down with it, so probe the socket instead.

    A display with a host part (ssh -X sets DISPLAY=localhost:10.0) has no
    local socket to probe; trust it rather than silently going headless.
    """
    disp = os.environ.get("DISPLAY", "")
    if not disp:
        return False
    host, _, screen = disp.rpartition(":")
    if host:
        return True
    return os.path.exists("/tmp/.X11-unix/X" + screen.split(".")[0])


def _headless_render_default(has_display: bool) -> str:
    """Default for the ``headless_rendering`` argument.

    gz-sim's sensor rendering and the GUI viewers are independent choices,
    and on the GPU host they must be: the viewers draw on the Xvfb desktop
    (llvmpipe), while the camera has to keep rendering through EGL on the
    NVIDIA device. TB3_HEADLESS_RENDERING (set to "true" in
    docker/compose.yaml) pins that, overriding the DISPLAY heuristic that
    is still right for a workstation with a real GPU-backed display.
    """
    override = os.environ.get("TB3_HEADLESS_RENDERING", "").strip().lower()
    if override in ("true", "false"):
        return override
    return "false" if has_display else "true"


_HAS_DISPLAY = _x_display_available()
_GUI_DEFAULT = "true" if _HAS_DISPLAY else "false"
_HEADLESS_RENDER_DEFAULT = _headless_render_default(_HAS_DISPLAY)


def generate_launch_description():
    # turtlebot3_gazebo's launch files read this at import time too, so an
    # unset value would fail there first; default to the model this project
    # standardises on (the Dockerfile also exports it).
    tb3_model = os.environ.get("TURTLEBOT3_MODEL", "waffle_pi")

    pkg_fe = get_package_share_directory("tb3_frontier_exploration")
    pkg_tb3_gz = get_package_share_directory("turtlebot3_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    robot_sdf = os.path.join(
        pkg_tb3_gz, "models", f"turtlebot3_{tb3_model}", "model.sdf"
    )
    bridge_config = os.path.join(pkg_fe, "config", "gz_bridge.yaml")

    # gz-sim resolves model:// URIs from GZ_SIM_RESOURCE_PATH (Classic used
    # GAZEBO_MODEL_PATH). This package's own models/ is added by its
    # ament environment hook; turtlebot3_gazebo's models/ holds the meshes
    # the robot SDF references as model://turtlebot3_common/...
    tb3_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_tb3_gz, "models")
    )

    def make_gz_server(context, *_args, **_kwargs):
        """Build the gz-sim server include, adding --headless-rendering when
        asked for.

        An OpaqueFunction rather than a substitution because gz_args is a
        single concatenated string: there is no clean way to conditionally
        splice a flag into it without resolving the LaunchConfiguration
        first.
        """
        headless = LaunchConfiguration("headless_rendering").perform(context)
        flag = "--headless-rendering " if headless.lower() == "true" else ""
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": ["-r -s -v2 ", flag, world],
                "on_exit_shutdown": "true",
            }.items(),
        )]

    # The GUI is a separate process in gz-sim, as gzclient was in Classic;
    # the launch argument keeps its old name so existing commands and docs
    # ("use_gzclient:=false" for headless runs) still work.
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g -v2 "}.items(),
        condition=IfCondition(LaunchConfiguration("use_gzclient")),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gz, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_turtlebot3",
        arguments=[
            "-name", tb3_model,
            "-file", robot_sdf,
            "-x", x_pose,
            "-y", y_pose,
            "-z", "0.01",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": use_sim_time}],
        output="screen",
    )

    # Images go through image_bridge rather than parameter_bridge so the
    # 640x480 RGB frames the detector consumes are converted once.
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="camera_bridge",
        arguments=["/camera/image_raw"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "use_gzclient", default_value=_GUI_DEFAULT,
            description="Launch the Gazebo GUI client. Defaults to true only "
                        "when an X server is reachable — on the GPU host "
                        "that means docker/start_gui.sh is running and the "
                        "GUI is viewable over noVNC.",
        ),
        DeclareLaunchArgument(
            "headless_rendering", default_value=_HEADLESS_RENDER_DEFAULT,
            description="Render camera/lidar sensors through EGL instead of an "
                        "X display. Forced by TB3_HEADLESS_RENDERING; "
                        "otherwise true when no X server is reachable.",
        ),
        DeclareLaunchArgument(
            "world", description="Absolute path to the .world (SDF) file to load",
        ),
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        tb3_resources,
        OpaqueFunction(function=make_gz_server),
        gz_gui,
        robot_state_publisher,
        spawn,
        bridge,
        image_bridge,
    ])

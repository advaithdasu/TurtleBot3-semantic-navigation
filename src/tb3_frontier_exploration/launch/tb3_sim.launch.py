"""
tb3_sim.launch.py — Gazebo Harmonic (gz-sim) + TurtleBot3 bringup.

Single entry point for "start the simulator with a robot in it", shared by
detector_test_sim, warehouse_semantic_sim and full_semantic_nav. Replaces
the Gazebo Classic gzserver.launch.py / gzclient.launch.py pair, which had
no arm64 binaries and forced the whole image through Rosetta.

Composes:
  1. gz-sim server (``-r -s``) on the requested world, plus the GUI client
     (``-g``) unless use_gzclient:=false
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
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s -v2 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

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
            "use_gzclient", default_value="true",
            description="Launch the Gazebo GUI client (false for headless runs)",
        ),
        DeclareLaunchArgument(
            "world", description="Absolute path to the .world (SDF) file to load",
        ),
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        tb3_resources,
        gz_server,
        gz_gui,
        robot_state_publisher,
        spawn,
        bridge,
        image_bridge,
    ])

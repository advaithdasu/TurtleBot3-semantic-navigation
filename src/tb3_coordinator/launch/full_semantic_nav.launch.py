"""
full_semantic_nav.launch.py — One-command launch for the full semantic navigation stack.

Composes:
  1. Gazebo simulation (warehouse_semantic world)
  2. SLAM Toolbox (online async)
  3. Nav2 navigation stack
  4. Frontier exploration (warmup + frontier detection + goal assignment)
  5. Semantic perception chain (detector + localizer + memory + query + nav_adapter)
  6. Coordinator (mode manager)

Usage:
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch tb3_coordinator full_semantic_nav.launch.py

    # Then in another terminal:
    ros2 topic pub --once /user_command std_msgs/String "data: 'go to the person'"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # ── Package share dirs ────────────────────────────────────────────────
    pkg_fe = get_package_share_directory("tb3_frontier_exploration")
    pkg_det = get_package_share_directory("tb3_detector")
    pkg_loc = get_package_share_directory("tb3_localizer")
    pkg_mem = get_package_share_directory("tb3_memory")
    pkg_qry = get_package_share_directory("tb3_query")
    pkg_nav = get_package_share_directory("tb3_nav_adapter")
    pkg_coord = get_package_share_directory("tb3_coordinator")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    # ── 1. Gazebo + TurtleBot3 ────────────────────────────────────────────
    # Use warehouse_semantic_models.world (real mesh models) instead of
    # warehouse_semantic.world (placeholder cylinders).
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    launch_tb3 = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    world = os.path.join(pkg_fe, "worlds", "warehouse_semantic_models.world")
    x_pose = LaunchConfiguration("x_pose", default="-1.2")
    y_pose = LaunchConfiguration("y_pose", default="-1.2")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_tb3, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_tb3, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )
    gazebo = [gzserver, gzclient, rsp, spawn]

    # ── 2. Nav2 + SLAM (integrated) ─────────────────────────────────────
    # Use Nav2 bringup with slam=True so it launches slam_toolbox internally
    # and does not require a pre-built map file.
    # Nav2 bringup: slam=True makes it launch slam_toolbox internally.
    # map arg is required even with slam=True; provide a dummy path that won't be loaded.
    # params_file must be explicit to avoid empty-path errors from ParameterFile.
    dummy_map = os.path.join(pkg_nav2, "maps", "turtlebot3_world.yaml")
    nav2_params = os.path.join(pkg_nav2, "params", "nav2_params.yaml")
    # Mute the very noisy `worldToMap failed: mx,my: ...` ERROR that
    # planner_server prints whenever the costmap inflation samples one cell
    # past the static-map boundary. It is benign in Nav2 Humble (planning
    # still succeeds and the goal is reached), but it floods the terminal
    # at planning rate.
    #
    # nav2_bringup forwards this single string verbatim as
    # `--ros-args --log-level <value>` to every Nav2 node. ROS 2's
    # `--log-level` supports a per-logger form `<logger>:=<level>`; when
    # passed a logger name that does not exist on the receiving node the
    # rcl logging machinery silently ignores it and the process default
    # stays at INFO. Therefore `planner_server:=fatal`:
    #   • on planner_server  → matches its node logger → silenced.
    #   • on every other Nav2 node (bt_navigator, controller_server, …)
    #     → no such logger → INFO defaults preserved, goal lifecycle and
    #     recoveries still print.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
            "map": dummy_map,
            "params_file": nav2_params,
            "autostart": "True",
            "use_composition": "False",
            "log_level": "planner_server:=fatal",
        }.items(),
    )

    # ── 4. Frontier exploration ───────────────────────────────────────────
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fe, "launch", "exploration.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── 5. Semantic perception chain ──────────────────────────────────────
    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_det, "launch", "detector.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_loc, "launch", "localizer.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    memory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mem, "launch", "semantic_memory.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    query = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_qry, "launch", "semantic_query.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    nav_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "nav_goal_adapter.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── 6. Coordinator ────────────────────────────────────────────────────
    coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_coord, "launch", "coordinator.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── Compose with startup delays ──────────────────────────────────────
    # Gazebo + SLAM + Nav2 need time to initialize before exploration and
    # perception nodes start looking for topics and action servers.
    # ── 7. RViz (optional) ─────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([
        FindPackageShare("tb3_coordinator"), "rviz", "semantic_nav.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with semantic nav config"),

        DeclareLaunchArgument("x_pose", default_value="-1.2"),
        DeclareLaunchArgument("y_pose", default_value="-1.2"),

        # Phase 1: simulation + navigation infrastructure
        # Nav2 (autostart=True) needs Gazebo + SLAM to publish /map before
        # planner_server can finish configure(); 10s is a safer buffer than 5s
        # to absorb CPU jitter (otherwise lifecycle_manager hangs on
        # "Waiting for service planner_server/get_state...").
        *gazebo,
        TimerAction(period=10.0, actions=[nav2]),

        # Phase 2: exploration + perception (after Nav2 has time to start)
        TimerAction(period=20.0, actions=[exploration]),
        TimerAction(period=15.0, actions=[detector]),
        TimerAction(period=15.0, actions=[localizer]),
        TimerAction(period=15.0, actions=[memory]),
        TimerAction(period=15.0, actions=[query]),
        TimerAction(period=15.0, actions=[nav_adapter]),

        # Phase 3: coordinator (after everything else is up)
        TimerAction(period=23.0, actions=[coordinator]),

        # Persistent semantic map memory + RViz visualization
        TimerAction(period=17.0, actions=[
            Node(
                package="tb3_coordinator",
                executable="semantic_map_memory_node",
                name="semantic_map_memory_node",
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare("tb3_coordinator"), "config", "coordinator.yaml"
                    ]),
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            ),
        ]),

        # RViz (immediate, gated by use_rviz arg)
        rviz_node,

        # Runtime debug diagnostics (gated by use_runtime_debug arg)
        DeclareLaunchArgument("use_runtime_debug", default_value="false",
                              description="Launch semantic runtime debug node"),
        TimerAction(period=17.0, actions=[
            Node(
                package="tb3_coordinator",
                executable="semantic_runtime_debug_node",
                name="semantic_runtime_debug_node",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("use_runtime_debug")),
                output="screen",
            ),
        ]),
    ])

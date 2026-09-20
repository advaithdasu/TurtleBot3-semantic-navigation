"""
detector_test_sim.launch.py — Gazebo Harmonic with detector_test.world for
Stage-1 YOLO testing.

Thin wrapper over tb3_sim.launch.py, which owns the gz-sim server/GUI,
spawn and ros_gz bridges.

Usage:
    export TURTLEBOT3_MODEL=waffle_pi   # burger has no camera
    ros2 launch tb3_frontier_exploration detector_test_sim.launch.py

Optional overrides:
    ros2 launch tb3_frontier_exploration detector_test_sim.launch.py \
        x_pose:=0.0 y_pose:=0.0 use_gzclient:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Matches tb3_sim.launch.py. This wrapper passes use_gzclient down
# explicitly, so a hardcoded "true" here would override tb3_sim's own
# default and try to open a GUI with no X server behind it. DISPLAY being
# set is not enough on the GPU host (compose exports DISPLAY=:99 whether or
# not docker/start_gui.sh has started the noVNC desktop), so probe the
# socket the same way tb3_sim.launch.py does.
def _x_display_available() -> bool:
    disp = os.environ.get("DISPLAY", "")
    if not disp:
        return False
    host, _, screen = disp.rpartition(":")
    if host:                       # ssh -X style DISPLAY=localhost:10.0
        return True
    return os.path.exists("/tmp/.X11-unix/X" + screen.split(".")[0])


_GUI_DEFAULT = "true" if _x_display_available() else "false"


def generate_launch_description():
    pkg_tb3_fe = get_package_share_directory("tb3_frontier_exploration")
    world = os.path.join(pkg_tb3_fe, "worlds", "detector_test.world")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_fe, "launch", "tb3_sim.launch.py")
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_gzclient": LaunchConfiguration("use_gzclient"),
            "x_pose": LaunchConfiguration("x_pose"),
            "y_pose": LaunchConfiguration("y_pose"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_gzclient", default_value=_GUI_DEFAULT,
                              description="Launch the Gazebo GUI client. Defaults "
                                          "to true when an X server is reachable."),
        # Robot spawns at origin facing +X so all test objects are directly ahead.
        DeclareLaunchArgument("x_pose", default_value="0.0",
                              description="TB3 spawn X (world frame)"),
        DeclareLaunchArgument("y_pose", default_value="0.0",
                              description="TB3 spawn Y (world frame)"),
        sim,
    ])

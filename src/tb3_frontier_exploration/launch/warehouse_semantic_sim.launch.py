"""
Gazebo Harmonic + TurtleBot3 in warehouse_semantic.world.

Thin wrapper over tb3_sim.launch.py, which owns the gz-sim server/GUI,
spawn and ros_gz bridges. Set TURTLEBOT3_MODEL before launch.
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
    world = os.path.join(pkg_tb3_fe, "worlds", "warehouse_semantic.world")

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
        # Defaults inside 4 m × 6 m floor (inner x ∈ [-2, 2], y ∈ [-3, 3]).
        DeclareLaunchArgument("x_pose", default_value="-1.2"),
        DeclareLaunchArgument("y_pose", default_value="-1.2"),
        sim,
    ])

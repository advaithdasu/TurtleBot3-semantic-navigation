"""
evidence_store.launch.py — Launch the Stage-5 evidence store node.

Usage:
    ros2 launch tb3_grounding evidence_store.launch.py
    ros2 launch tb3_grounding evidence_store.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_grounding")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running with Gazebo simulation.",
        ),

        Node(
            package="tb3_grounding",
            executable="evidence_store_node",
            name="evidence_store_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "grounding.yaml"]),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])

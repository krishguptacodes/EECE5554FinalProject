from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file to start the SLAM monitor node.

    runs on top of our normal TurtleBot3 SLAM launch.
    """
    return LaunchDescription(
        [
            Node(
                package="slam_tools",
                executable="slam_monitor_node.py",
                name="slam_monitor",
                output="screen",
            ),
        ]
    )

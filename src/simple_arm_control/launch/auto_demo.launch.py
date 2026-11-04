from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory("simple_arm_moveit_config")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(moveit_share) / "launch" / "demo.launch.py")
        )
    )

    move_to_ready = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="simple_arm_control",
                executable="move_arm",
                output="screen",
                parameters=[{"target_name": "ready"}],
            )
        ],
    )
    return LaunchDescription([simulation, move_to_ready])

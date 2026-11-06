from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("robot_arm_description"))
    urdf_path = package_share / "urdf" / "generic_6dof_arm_with_gripper.urdf"

    return LaunchDescription(
        [
            Node(
                package="moveit_setup_assistant",
                executable="moveit_setup_assistant",
                arguments=["--urdf_path", str(urdf_path)],
                output="screen",
            )
        ]
    )

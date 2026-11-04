from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = (
        Path(get_package_share_directory("simple_arm_moveit_config"))
        / "config"
        / "kinematics.yaml"
    )
    with config_path.open(encoding="utf-8") as stream:
        kinematics = yaml.safe_load(stream)

    return LaunchDescription([
        Node(
            package="simple_arm_gui",
            executable="simple_arm_gui_node",
            output="screen",
            parameters=[{"robot_description_kinematics": kinematics}],
        )
    ])

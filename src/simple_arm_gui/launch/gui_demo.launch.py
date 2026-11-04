from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory("simple_arm_moveit_config")
    use_rviz = LaunchConfiguration("use_rviz")
    with (Path(moveit_share) / "config" / "kinematics.yaml").open(encoding="utf-8") as stream:
        kinematics = yaml.safe_load(stream)

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(moveit_share) / "launch" / "demo.launch.py")
        ),
        launch_arguments={"use_rviz": use_rviz}.items(),
    )

    gui = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="simple_arm_gui",
                executable="simple_arm_gui_node",
                output="screen",
                parameters=[{"robot_description_kinematics": kinematics}],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start RViz together with the Chinese GUI",
        ),
        simulation,
        gui,
    ])

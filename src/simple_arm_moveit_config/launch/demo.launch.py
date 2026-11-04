from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    path = Path(get_package_share_directory(package_name)) / relative_path
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_text(package_name, relative_path):
    path = Path(get_package_share_directory(package_name)) / relative_path
    return path.read_text(encoding="utf-8")


def generate_launch_description():
    description_share = get_package_share_directory("simple_arm_description")
    moveit_share = get_package_share_directory("simple_arm_moveit_config")

    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([description_share, "urdf", "simple_arm.urdf.xacro"]),
            " use_mock_hardware:=true",
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {
        "robot_description_semantic": load_text(
            "simple_arm_moveit_config", "config/simple_arm.srdf"
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "simple_arm_moveit_config", "config/kinematics.yaml"
        )
    }
    joint_limits = load_yaml("simple_arm_moveit_config", "config/joint_limits.yaml")
    ompl_pipeline = load_yaml("simple_arm_moveit_config", "config/ompl_planning.yaml")
    controllers = load_yaml("simple_arm_moveit_config", "config/moveit_controllers.yaml")

    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        joint_limits,
        ompl_pipeline,
        controllers,
        {
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "allow_trajectory_execution": True,
            "moveit_manage_controllers": False,
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.01,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
    ]

    use_rviz = LaunchConfiguration("use_rviz")
    use_move_group = LaunchConfiguration("use_move_group")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true", description="Start RViz with MoveIt panel"
        ),
        DeclareLaunchArgument(
            "use_move_group", default_value="true", description="Start MoveIt move_group"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                robot_description,
                str(Path(moveit_share) / "config" / "ros2_controllers.yaml"),
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            condition=IfCondition(use_move_group),
            output="screen",
            parameters=move_group_parameters,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            condition=IfCondition(use_rviz),
            arguments=["-d", str(Path(moveit_share) / "rviz" / "moveit.rviz")],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                joint_limits,
                ompl_pipeline,
            ],
        ),
    ])

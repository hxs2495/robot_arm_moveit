from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    gazebo_share = get_package_share_directory("simple_arm_gazebo")
    gazebo_prefix = get_package_prefix("simple_arm_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    moveit_share = get_package_share_directory("simple_arm_moveit_config")

    use_rviz = LaunchConfiguration("use_rviz")
    use_perception = LaunchConfiguration("use_perception")
    use_gui = LaunchConfiguration("use_gui")
    gazebo_gui = LaunchConfiguration("gui")
    auto_pick = LaunchConfiguration("auto_pick")
    pick_x = LaunchConfiguration("pick_x")
    pick_y = LaunchConfiguration("pick_y")
    place_x = LaunchConfiguration("place_x")
    place_y = LaunchConfiguration("place_y")
    world = LaunchConfiguration("world")
    controllers_file = str(Path(gazebo_share) / "config" / "gazebo_controllers.yaml")

    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([description_share, "urdf", "simple_arm.urdf.xacro"]),
            " use_mock_hardware:=false",
            " use_gazebo:=true",
            " enable_gripper_plugin:=true",
            " enable_sensors:=true",
            " controllers_file:=", controllers_file,
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
    sensors_3d = load_yaml("simple_arm_gazebo", "config/sensors_3d.yaml")

    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        joint_limits,
        ompl_pipeline,
        controllers,
        sensors_3d,
        {
            "use_sim_time": True,
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "octomap_frame": "world",
            "octomap_resolution": 0.03,
            "max_range": 4.0,
            "allow_trajectory_execution": True,
            "moveit_manage_controllers": False,
            "trajectory_execution.allowed_execution_duration_scaling": 1.5,
            "trajectory_execution.allowed_goal_duration_margin": 1.0,
            "trajectory_execution.allowed_start_tolerance": 0.02,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(gazebo_ros_share) / "launch" / "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gazebo_gui,
            "verbose": "false",
            "pause": "false",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "simple_arm",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(period=0.2, actions=[joint_state_spawner]),
                TimerAction(period=0.4, actions=[arm_controller_spawner]),
            ],
        )
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_parameters,
    )
    feedback = Node(
        package="simple_arm_gazebo",
        executable="arm_state_feedback",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    recognition_server = Node(
        package="simple_arm_gazebo",
        executable="object_recognition_server",
        output="screen",
        condition=IfCondition(use_perception),
        parameters=[{"use_sim_time": True}],
    )
    pick_place_server = TimerAction(
        period=8.0,
        actions=[Node(
            package="simple_arm_gazebo",
            executable="pick_place_server",
            output="screen",
            condition=IfCondition(use_perception),
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                joint_limits,
                {"use_sim_time": True},
            ],
        )],
    )
    control_gui = TimerAction(
        period=10.0,
        actions=[Node(
            package="simple_arm_gui",
            executable="simple_arm_gui_node",
            output="screen",
            condition=IfCondition(use_gui),
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                {"use_sim_time": True},
            ],
        )],
    )
    rviz = Node(
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
            {"use_sim_time": True},
        ],
    )
    pick_demo = TimerAction(
        period=10.0,
        actions=[Node(
            package="simple_arm_gazebo",
            executable="pick_place_demo",
            output="screen",
            condition=IfCondition(auto_pick),
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                {
                    "use_sim_time": True,
                    "pick_x": ParameterValue(pick_x, value_type=float),
                    "pick_y": ParameterValue(pick_y, value_type=float),
                    "place_x": ParameterValue(place_x, value_type=float),
                    "place_y": ParameterValue(place_y, value_type=float),
                },
            ],
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=str(Path(gazebo_share) / "worlds" / "pick_place.world"),
            description="Gazebo world file",
        ),
        DeclareLaunchArgument("gui", default_value="true", description="Open Gazebo GUI"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Open RViz MoveIt panel"),
        DeclareLaunchArgument("use_perception", default_value="true", description="Start sensor perception and task servers"),
        DeclareLaunchArgument("use_gui", default_value="false", description="Open the Chinese Qt task console"),
        DeclareLaunchArgument("auto_pick", default_value="false", description="Run pick-place demo automatically"),
        DeclareLaunchArgument("pick_x", default_value="0.50", description="Pick position X in world frame"),
        DeclareLaunchArgument("pick_y", default_value="0.20", description="Pick position Y in world frame"),
        DeclareLaunchArgument("place_x", default_value="0.50", description="Place position X in world frame"),
        DeclareLaunchArgument("place_y", default_value="-0.20", description="Place position Y in world frame"),
        AppendEnvironmentVariable(
            name="GAZEBO_PLUGIN_PATH", value=str(Path(gazebo_prefix) / "lib")
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        start_controllers,
        move_group,
        feedback,
        recognition_server,
        pick_place_server,
        control_gui,
        rviz,
        pick_demo,
    ])

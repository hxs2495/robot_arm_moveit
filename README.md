# ROS 2 Humble + MoveIt 2 六轴机械臂仿真

本工作区提供一个无外部模型依赖的六轴机械臂，可先通过 RViz、MoveIt 2 和
`ros2_control` mock hardware 完成规划与轨迹执行。后续接入 Gazebo 或真实机械臂时，
可保留 MoveIt 配置，只替换 URDF 尺寸、网格和 `ros2_control` hardware plugin。

## 功能包

- `simple_arm_description`：六轴机械臂 Xacro、碰撞体和 ros2_control 接口。
- `simple_arm_moveit_config`：SRDF、KDL、OMPL、控制器及 RViz 一键启动。
- `simple_arm_control`：基于 `MoveGroupInterface` 的关节目标控制示例。
- `simple_arm_gui`：独立的全中文 Qt 机械臂控制与状态管理界面。

## 安装依赖与构建

```bash
cd /home/hxs/open_source_project/robot_arm_moveit
./scripts/build.sh
source install/setup.bash
```

如果系统尚未安装 MoveIt 2，`rosdep` 会安装声明的二进制依赖。也可手动安装：

```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-moveit-kinematics
```

## 启动仿真

```bash
./scripts/run_sim.sh
```

在 RViz 的 MotionPlanning 面板中：

1. Planning Group 选择 `arm`。
2. 拖动末端交互标记，或在 Goal State 选择 `ready`。
3. 点击 **Plan & Execute**，机械臂会按规划轨迹运动。

自动执行一次 `ready` 姿态：

```bash
./scripts/run_auto_demo.sh
```

仿真保持运行时，可在另一终端发送命名目标或六个关节角（弧度）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run simple_arm_control move_arm --ros-args -p target_name:=home
ros2 run simple_arm_control move_arm --ros-args \
  -p joint_positions:="[0.4, -0.7, 1.0, 0.2, -0.3, 0.0]"
```

仅启动后台、不打开 RViz：

```bash
./scripts/run_sim.sh use_rviz:=false
```

## 中文 Qt 控制界面

一键启动仿真、RViz 和中文控制界面：

```bash
./scripts/run_gui_demo.sh
```

不需要 RViz 时：

```bash
./scripts/run_gui_demo.sh use_rviz:=false
```

如果仿真或真实机械臂后台已经启动，可只打开控制界面：

```bash
./scripts/run_gui.sh
```

界面提供六个关节的实时角度、目标角度、零位/准备位、规划、执行、规划并执行、停止运动、速度和加速度限制以及中文运行日志。

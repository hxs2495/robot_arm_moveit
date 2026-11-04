# ROS 2 Humble + MoveIt 2 六轴机械臂仿真

本工作区提供一个无外部模型依赖的六轴机械臂，可先通过 RViz、MoveIt 2 和
`ros2_control` mock hardware 完成规划与轨迹执行。后续接入 Gazebo 或真实机械臂时，
可保留 MoveIt 配置，只替换 URDF 尺寸、网格和 `ros2_control` hardware plugin。

## 功能包

- `simple_arm_description`：六轴机械臂 Xacro、碰撞体和 ros2_control 接口。
- `simple_arm_moveit_config`：SRDF、KDL、OMPL、控制器及 RViz 一键启动。
- `simple_arm_control`：基于 `MoveGroupInterface` 的关节目标控制示例。
- `simple_arm_gui`：独立的全中文 Qt 机械臂控制与状态管理界面。
- `simple_arm_gazebo`：Gazebo Classic 物理仿真、ros2_control、状态反馈与 MoveIt 自动抓放。

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
  ros-humble-ros2-controllers ros-humble-moveit-kinematics \
  ros-humble-moveit-ros-perception ros-humble-gazebo-plugins
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


## Gazebo 物理仿真与抓取

新功能包使用 Gazebo Classic 11、`gazebo_ros2_control` 和 MoveIt 2。Gazebo 中包含固定工作台、可抓取方块和六轴机械臂；MoveIt 规划结果通过 `arm_controller/follow_joint_trajectory` 在物理仿真中执行。

只启动 Gazebo、MoveIt 和 RViz，之后可在 RViz 的 MotionPlanning 面板中交互规划：

```bash
./scripts/run_gazebo.sh
```

一键运行完整抓取放置演示：

```bash
./scripts/run_gazebo_pick.sh
```

抓取流程为：准备位 → 预抓取 → 垂直下降 → 附着方块 → 抬升 → 搬运 → 放置 → 释放 → 返回准备位。MoveIt 规划场景同步包含工作台和方块，Gazebo 夹具插件负责物理对象的附着与释放。

无图形界面运行：

```bash
./scripts/run_gazebo_pick.sh gui:=false use_rviz:=false
```

需要修改取放坐标时，可直接覆盖启动参数（坐标使用 `world` 坐标系，单位为米）：

```bash
./scripts/run_gazebo_pick.sh \
  pick_x:=0.50 pick_y:=0.20 \
  place_x:=0.50 place_y:=-0.20
```

主要控制与反馈接口：

- `/arm_controller/follow_joint_trajectory`：MoveIt 使用的轨迹动作接口。
- `/joint_states`：Gazebo ros2_control 原始关节反馈。
- `/simple_arm/feedback/joint_states`：功能包输出的关节状态反馈。
- `/simple_arm/feedback/tool_pose`：`world` 坐标系下的末端位姿反馈。
- `/simple_arm/gripper_command`：夹具附着/释放命令，`std_msgs/Bool`。

查看反馈：

```bash
ros2 control list_controllers
ros2 topic echo /simple_arm/feedback/joint_states
ros2 topic echo /simple_arm/feedback/tool_pose
```

启动参数包括 `gui`、`use_rviz`、`auto_pick` 和 `world`，例如只打开 Gazebo：

```bash
./scripts/run_gazebo.sh use_rviz:=false
```

## 三维感知与 GUI 抓取闭环

Gazebo 模型现在包含固定式 RGB-D 深度相机和 3D 激光雷达。两路点云会由
MoveIt 的 Occupancy Map Monitor 融合到 OctoMap；固定传感器链通过
`/tf_static` 发布，六轴机械臂运动链通过 `/tf` 动态更新。

一键启动 Gazebo、MoveIt、RViz、感知节点和中文抓取管理界面：

```bash
./scripts/run_gazebo_gui.sh
```

界面中的“智能抓取业务闭环”支持：

1. 查看深度相机、3D 雷达和 OctoMap 在线状态。
2. 从深度点云刷新并选择识别目标。
3. 输入目标放置点的 world 坐标。
4. 下发抓取任务，实时查看规划、接近、夹取、搬运和放置阶段。
5. 在执行过程中取消任务。

主要感知与业务接口：

- `/simple_arm/sensors/depth_camera/image_raw`：RGB 图像。
- `/simple_arm/sensors/depth_camera/depth/image_raw`：深度图像。
- `/simple_arm/sensors/depth_camera/points`：RGB-D 点云。
- `/simple_arm/sensors/lidar/points`：3D 激光雷达点云。
- `/simple_arm/perception/detections`：标准 `vision_msgs/Detection3DArray` 识别结果。
- `/simple_arm/perception/detect_objects`：刷新目标识别服务。
- `/simple_arm/tasks/pick_place`：带阶段反馈和取消能力的抓放 Action。

常用检查命令：

```bash
ros2 topic hz /simple_arm/sensors/depth_camera/points
ros2 topic hz /simple_arm/sensors/lidar/points
ros2 run tf2_ros tf2_echo world depth_camera_optical_frame
ros2 run tf2_ros tf2_echo world lidar_link
ros2 service call /simple_arm/perception/detect_objects \
  simple_arm_interfaces/srv/DetectObjects "{refresh: true}"
```

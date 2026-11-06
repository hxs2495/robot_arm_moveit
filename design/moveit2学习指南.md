# MoveIt 2 从入门到实机：六轴机械臂与夹爪实践教程

> 适用环境：Ubuntu 22.04、ROS 2 Humble、MoveIt 2.5.x、ros2_control 2.x  
> 配套工程：`robot_arm_moveit`  
> 配套模型：`generic_6dof_arm_with_gripper`  
> 文档版本：2026-11-06

这不是一份只介绍按钮位置的说明书，而是一条完整的学习路线：先理解 MoveIt 2 在机器人系统中的位置，再创建并校验 URDF，使用 Setup Assistant 配置机械臂和夹爪，运行 RViz Demo，最后把 Mock Hardware 替换成真实机械臂驱动。

---

## 目录

1. [先建立整体认识](#1-先建立整体认识)
2. [准备环境与工作区](#2-准备环境与工作区)
3. [创建可用于 MoveIt 的 URDF](#3-创建可用于-moveit-的-urdf)
4. [使用 Setup Assistant 生成配置](#4-使用-setup-assistant-生成配置)
5. [补全 ros2_control 与夹爪控制](#5-补全-ros2_control-与夹爪控制)
6. [运行 Demo 并完成第一次规划](#6-运行-demo-并完成第一次规划)
7. [用 C++ 调用 MoveIt](#7-用-c-调用-moveit)
8. [规划场景、抓取与感知](#8-规划场景抓取与感知)
9. [从 Mock Hardware 切换到实机](#9-从-mock-hardware-切换到实机)
10. [常见故障与定位方法](#10-常见故障与定位方法)
11. [推荐练习路线](#11-推荐练习路线)
12. [术语与参考资料](#12-术语与参考资料)

建议按下面的节奏学习：

| 阶段 | 建议用时 | 完成标志 |
|---|---:|---|
| 框架与 URDF | 0.5～1 天 | 能解释 URDF、SRDF、`move_group`、ros2_control 的关系 |
| Setup Assistant | 0.5 天 | 能独立生成并读懂 `*_moveit_config` |
| RViz 与编程 | 1～2 天 | 能规划、执行、添加障碍物 |
| 夹爪与抓取 | 1～2 天 | 能控制平行夹爪并完成 attach/detach |
| 实机联动 | 视驱动而定 | 编码器反馈、轨迹执行和急停链路全部通过验收 |

实际操作时始终按下面的顺序推进，不要跳过中间验收点：

1. 固定 ROS、MoveIt、Xacro 和 ros2_control 版本；
2. 创建 URDF，并把关节限位明确写成浮点数；
3. 用 `check_urdf` 验证模型；
4. 在 Setup Assistant 中按页面顺序配置 SRDF；
5. 生成配置包，但把生成结果视为“初稿”；
6. 检查 YAML 数值类型、控制器 Action 和 SRDF 交叉关系；
7. 构建并启动 Mock Hardware Demo；
8. 分别测试机械臂和夹爪控制器；
9. 再编写规划、抓取和 PlanningScene 程序；
10. 最后才替换成实机 HardwareInterface。

---

## 1. 先建立整体认识

### 1.1 MoveIt 2 到底负责什么

一句话概括：

> MoveIt 2 负责“机器人应该怎样运动”，ros2_control 和硬件驱动负责“电机怎样真正动起来”。

MoveIt 2 的核心工作包括：

- 维护机器人当前状态；
- 维护机器人与环境的碰撞模型；
- 进行正运动学、逆运动学和碰撞检测；
- 根据目标位姿或目标关节角规划路径；
- 给路径增加速度、加速度和时间信息；
- 把最终轨迹交给控制器执行。

MoveIt 2 **不是**伺服驱动器，也不会直接发送 CAN、EtherCAT 或串口报文。它通常把 `trajectory_msgs/JointTrajectory` 包装在 `FollowJointTrajectory` Action 中，交给 ros2_control 控制器。

![MoveIt 2 核心架构](assets/moveit2_architecture.svg)

图中的主数据流可以读成：

```text
目标位姿
  → move_group
  → IK + PlanningScene + Planner
  → 带时间的关节轨迹
  → MoveIt Controller Manager
  → JointTrajectoryController
  → ros2_control HardwareInterface
  → 电机

编码器
  → HardwareInterface
  → /joint_states
  → MoveIt CurrentStateMonitor
```

### 1.2 最重要的组件

| 组件 | 通俗解释 | 本工程中的位置 |
|---|---|---|
| URDF/Xacro | 机器人的“骨架、外形、质量说明书” | `robot_arm_description/urdf` |
| SRDF | MoveIt 的“语义说明书” | `robot_arm_moveit_config/config/*.srdf` |
| RobotModel | URDF + SRDF 解析后的只读模型 | MoveIt 内部 |
| RobotState | 某一时刻所有关节值和连杆位姿 | MoveIt 内部 |
| PlanningScene | RobotState + 障碍物 + Allowed Collision Matrix | `move_group` 内部 |
| PlanningSceneMonitor | 持续同步关节、TF、碰撞物、OctoMap | `move_group` 内部 |
| Kinematics Plugin | FK/IK 求解器，如 KDL、IKFast | `kinematics.yaml` |
| Planning Pipeline | OMPL、Pilz、CHOMP 等规划器与适配器 | `ompl_planning.yaml` 等 |
| TrajectoryExecutionManager | 选择控制器并监督轨迹执行 | `move_group` 内部 |
| Controller Manager | 加载和管理 ros2_control 控制器 | `ros2_control_node` |
| HardwareInterface | Mock、仿真或真实硬件的统一抽象 | URDF 的 `<ros2_control>` |

官方的 [MoveIt Concepts](https://moveit.picknik.ai/humble/doc/concepts/concepts.html)、[Motion Planning](https://moveit.picknik.ai/humble/doc/concepts/motion_planning.html) 和 [move_group](https://moveit.picknik.ai/humble/doc/concepts/move_group.html) 页面适合在完成本章后继续阅读。

### 1.3 URDF 与 SRDF 的区别

URDF 描述“物理上有什么”：

- link 和 joint；
- visual 和 collision；
- 惯量、质量；
- 关节上下限；
- mimic 关系；
- ros2_control 接口。

SRDF 描述“MoveIt 应该怎样理解它”：

- 规划组；
- 虚拟关节；
- 命名姿态；
- 末端执行器；
- 被动关节；
- 可以忽略的自碰撞对。

两者缺一不可。官方说明见 [URDF and SRDF](https://moveit.picknik.ai/humble/doc/examples/urdf_srdf/urdf_srdf_tutorial.html)。

---

## 2. 准备环境与工作区

### 2.1 安装核心软件

```bash
sudo apt update
sudo apt install \
  ros-humble-moveit \
  ros-humble-moveit-setup-assistant \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui
```

进入当前工程：

```bash
cd ~/open_source_project/robot_arm_moveit
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

每开一个新终端，都要重新执行：

```bash
source /opt/ros/humble/setup.bash
source ~/open_source_project/robot_arm_moveit/install/setup.bash
```

“构建了但没 source”是 ROS 2 最常见的问题之一。

### 2.2 安装后先记录版本基线

MoveIt 2 不是单一软件包，它会同时使用 MoveIt、Xacro、ros2_control、
ros2_controllers 和 RViz。即使都属于 Humble，不同补丁版本或 backport 的发布时间也
可能不同，因此 Setup Assistant 的生成模板与运行时警告不一定完全同步。

安装后先记录版本：

```bash
dpkg-query -W \
  ros-humble-moveit \
  ros-humble-moveit-setup-assistant \
  ros-humble-xacro \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

本教程在以下主版本组合上完成过实际验证：

| 组件 | 验证版本 |
|---|---|
| ROS 2 | Humble / Ubuntu 22.04 |
| MoveIt 2 | 2.5.10 |
| ros2_control | 2.53.x |
| ros2_controllers | 2.52.x |

版本使用原则：

- 不要在同一个终端混用 Humble 的 apt 包和 Rolling/main 分支源码；
- source 顺序固定为 `/opt/ros/humble/setup.bash`，再 source 当前工作区；
- 升级 apt 包后重新构建工作区，并重新执行本教程第 4.13 节验收；
- 出现弃用警告时先看运行是否正常，再检查生成模板是否使用旧 API；
- 报错时把上面的版本输出和最早异常一起保存，不要只截 RViz 最后一条错误。

> Setup Assistant 是与当前安装版本配套的配置生成器，但它不会替你完成运行时集成测试。
> 生成文件仍需要人工检查，尤其是严格类型 YAML 和插件专用字段。

### 2.3 本教程使用的两个包

```text
src/
├── robot_arm_description/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── setup_assistant.launch.py
│   └── urdf/
│       └── generic_6dof_arm_with_gripper.urdf
└── robot_arm_moveit_config/
    ├── .setup_assistant
    ├── config/
    │   ├── generic_6dof_arm_with_gripper.srdf
    │   ├── generic_6dof_arm_with_gripper.urdf.xacro
    │   ├── generic_6dof_arm_with_gripper.ros2_control.xacro
    │   ├── joint_limits.yaml
    │   ├── kinematics.yaml
    │   ├── moveit_controllers.yaml
    │   └── ros2_controllers.yaml
    └── launch/
        ├── demo.launch.py
        ├── move_group.launch.py
        └── moveit_rviz.launch.py
```

确认 ROS 能发现两个包：

```bash
colcon list | grep robot_arm
ros2 pkg prefix robot_arm_description
ros2 pkg prefix robot_arm_moveit_config
```

期望两个 `ros2 pkg prefix` 都返回当前工作区的 `install/...` 路径。

---

## 3. 创建可用于 MoveIt 的 URDF

### 3.1 先规定坐标、单位和命名

推荐一开始就统一：

- 长度：米；
- 质量：千克；
- 角度：弧度；
- 关节轴遵循右手定则；
- 固定基座使用 `base_link`；
- 法兰使用 `tool0`；
- 任务坐标使用 `gripper_tcp` 或 `tcp_link`；
- 机械臂关节使用稳定、唯一的名称，如 `joint_1` 到 `joint_6`。

关节名会同时出现在 URDF、SRDF、控制器 YAML、硬件驱动和标定数据中。项目进入实机阶段后，随意改名的代价非常高。

### 3.2 link 的三个几何概念

```xml
<link name="link_2">
  <inertial>
    <origin xyz="0.13 0 0"/>
    <mass value="3.0"/>
    <inertia ixx="0.010" ixy="0" ixz="0"
             iyy="0.030" iyz="0" izz="0.028"/>
  </inertial>

  <visual>
    <origin xyz="0.13 0 0"/>
    <geometry>
      <box size="0.26 0.075 0.075"/>
    </geometry>
  </visual>

  <collision>
    <origin xyz="0.13 0 0"/>
    <geometry>
      <box size="0.26 0.075 0.075"/>
    </geometry>
  </collision>
</link>
```

- `visual`：只影响“看起来怎样”；
- `collision`：影响碰撞检测速度和结果；
- `inertial`：影响物理仿真和动力学。

视觉网格可以精细，碰撞网格应该尽量简单。MoveIt 官方建议整个机器人碰撞网格保持在数千个三角面量级，复杂 CAD 网格应先做简化。

### 3.3 joint 的关键字段

```xml
<joint name="joint_2" type="revolute">
  <parent link="link_1"/>
  <child link="link_2"/>
  <origin xyz="0 0 0.15"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.2" upper="2.2"
         effort="80.0" velocity="2.0"/>
  <dynamics damping="0.2" friction="0.05"/>
</joint>
```

重点检查：

1. `origin` 是否从父 link 坐标系正确指向关节；
2. `axis` 是否符合真实电机正方向；
3. `lower/upper` 是否与机械限位一致；
4. `velocity` 是不是用 rad/s；
5. revolute、continuous、prismatic 类型是否正确。

#### 关节限位统一使用显式浮点写法

URDF 是 XML，属性值首先是字符串；URDF 解析器会根据字段把 `2` 和 `2.0` 都转换为
浮点数。但是 Setup Assistant 还会把这些值重新写入 YAML，而 ROS 2 参数会严格区分
YAML 的 integer 和 double。因此，所有可能进入运行参数的关节限位建议统一写成：

```xml
<limit lower="-2.2" upper="2.2"
       effort="80.0" velocity="2.0"/>
```

不要写成：

```xml
<limit lower="-2.2" upper="2.2"
       effort="80" velocity="2"/>
```

本工程已经把 6 个机械臂关节和 2 个手指关节的 `lower`、`effort`、`velocity`
等整数字面量规范为带小数点的写法。需要注意：

- 这能表达清楚浮点意图，但不能保证所有 YAML 生成器都保留末尾 `.0`；
- URDF 标准没有 MoveIt 的 `max_acceleration` 字段；
- Setup Assistant 生成的 `max_acceleration: 0` 仍必须在生成后改为 `0.0`；
- 因此第 4.13 节的全量 YAML 类型检查仍然不能省略。

### 3.4 平行夹爪与 mimic joint

当前模型用一个主动指关节带动另一个从动指关节：

```xml
<joint name="left_finger_joint" type="prismatic">
  <parent link="gripper_base"/>
  <child link="left_finger"/>
  <origin xyz="0 0.052 0.045"/>
  <axis xyz="0 1 0"/>
  <limit lower="0.0" upper="0.025" effort="15.0" velocity="0.05"/>
</joint>

<joint name="right_finger_joint" type="prismatic">
  <parent link="gripper_base"/>
  <child link="right_finger"/>
  <origin xyz="0 -0.052 0.045"/>
  <axis xyz="0 -1 0"/>
  <limit lower="0.0" upper="0.025" effort="15.0" velocity="0.05"/>
  <mimic joint="left_finger_joint" multiplier="1.0" offset="0.0"/>
</joint>
```

这里的“主动”只表示命令接口由 `left_finger_joint` 接收。`right_finger_joint` 的位置由 mimic 关系计算。根据 [ros2_control mimic joint 文档](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/joints_userdoc.html)，mimic joint 不应再声明 command interface，但可以声明 state interface。

### 3.5 当前模型的运动树

下面这张图由 `urdf_to_graphviz` 直接从当前 URDF 生成，不是手工绘制：

![当前机械臂 URDF 运动树](assets/robot_arm_kinematic_tree.svg)

你应该能看出：

- `base_link → link_1 → ... → link_6` 是六轴串联链；
- `tool0` 和 `gripper_base` 通过固定关节连接；
- 两个手指从 `gripper_base` 分叉；
- `gripper_tcp` 是抓取任务使用的固定参考点。

### 3.6 description 包必须是 ament 包

`package.xml` 至少要包含：

```xml
<buildtool_depend>ament_cmake</buildtool_depend>

<export>
  <build_type>ament_cmake</build_type>
</export>
```

`CMakeLists.txt` 至少要安装 URDF 并调用 `ament_package()`：

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

如果只有 `package.xml`，但 build type 是普通 `cmake`，Setup Assistant 仍然会报：

```text
ament_index_cpp::PackageNotFoundError:
package 'robot_arm_description' not found
```

### 3.7 每次修改 URDF 后都要做的检查

```bash
cd ~/open_source_project/robot_arm_moveit
source /opt/ros/humble/setup.bash

xmllint --noout \
  src/robot_arm_description/urdf/generic_6dof_arm_with_gripper.urdf

check_urdf \
  src/robot_arm_description/urdf/generic_6dof_arm_with_gripper.urdf

# 人工确认所有 limit/mimic 数值使用明确的小数写法
rg '<limit|<mimic' \
  src/robot_arm_description/urdf/generic_6dof_arm_with_gripper.urdf

colcon build \
  --packages-select robot_arm_description \
  --symlink-install

source install/setup.bash
ros2 pkg prefix robot_arm_description
```

如果使用 Xacro，还要先展开：

```bash
xacro path/to/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

URDF 验收清单：

- [ ] XML 能解析；
- [ ] 只有一个根 link；
- [ ] 所有关节 parent/child 存在；
- [ ] 非 fixed joint 都有限位；
- [ ] 关节 limit 和 mimic 的数值使用 `2.0`、`0.0`、`1.0` 这类显式浮点写法；
- [ ] 初始姿态没有明显自碰撞；
- [ ] visual 与 collision 大致重合；
- [ ] 关节正方向与实机一致；
- [ ] TCP 坐标轴方向已经明确；
- [ ] description 包能被 `ros2 pkg prefix` 找到。

---

## 4. 使用 Setup Assistant 生成配置

官方参考：[MoveIt Setup Assistant](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)。

### 4.1 启动方式

当前工程已经提供“直接加载模型”的启动文件：

```bash
cd ~/open_source_project/robot_arm_moveit
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_description setup_assistant.launch.py
```

也可以使用原始入口：

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

然后选择：

```text
src/robot_arm_description/urdf/generic_6dof_arm_with_gripper.urdf
```

![Setup Assistant 已加载当前机械臂](assets/setup_assistant_overview.png)

![image-20261106140045429](assets/image-20261106140045429.png)

如果选择文件后界面卡住，先用 `check_urdf` 验证模型；如果 URDF 使用了
`package://...` 资源，还要确认相关 ament 包已经构建并 source。

### 4.2 推荐配置顺序

![Setup Assistant 推荐流程](assets/setup_assistant_workflow.svg)

按左侧页面从上到下操作，每完成一页就做一次小检查：

| 顺序 | 页面 | 本项目操作 | 离开页面前确认 |
|---:|---|---|---|
| 1 | Self-Collisions | 生成自碰撞矩阵 | 没有随意禁用可能发生的碰撞 |
| 2 | Virtual Joints | `world` 到 `base_link`，类型 fixed | child link 是机器人根链接 |
| 3 | Planning Groups | 创建 `arm` Chain | `base_link → tool0` 完整存在 |
| 4 | Planning Groups | 创建 `gripper` | 主动关节是 `left_finger_joint` |
| 5 | Robot Poses | 创建 `home/ready/open/closed` | 姿态值在 URDF 限位内 |
| 6 | End Effectors | 创建 `gripper_eef` | `tool0` 确实属于 `arm` |
| 7 | Passive Joints | 本模型通常留空 | mimic 关节不要重复当作被动关节 |
| 8 | ros2_control URDF | 选择 position 命令接口 | 右指不创建独立 command interface |
| 9 | ROS 2 Controllers | 创建 arm 与 gripper 控制器 | 控制器关节集合没有重叠冲突 |
| 10 | Configuration Files | 生成配置包 | 继续执行 4.13 节，而不是直接运行 Demo |

### 4.3 Self-Collisions

进入 **Self-Collisions**，生成 Self-Collision Matrix。

它的作用是跳过不可能碰撞或永远相邻的 link 对，从而提高规划速度。建议：

- 初学与小模型：10,000～50,000 次采样；
- 正式发布：100,000 次或更多；
- 修改 link、collision geometry、关节限位后重新生成。

不要为了“让规划通过”而随便禁用真实可能发生的碰撞。

![image-20261106140120904](assets/image-20261106140120904.png)

### 4.4 Virtual Joints

固定式机械臂推荐：

| 字段 | 值 |
|---|---|
| Virtual Joint Name | `virtual_joint` |
| Child Link | `base_link` |
| Parent Frame | `world` |
| Joint Type | `fixed` |

如果机械臂安装在移动底盘上，应根据系统设计选择 planar 或 floating，并由定位系统持续发布对应 TF。

![image-20261106140301028](assets/image-20261106140301028.png)

![image-20261106140311317](assets/image-20261106140311317.png)

### 4.5 配置机械臂 Planning Group

推荐配置：

| 字段 | 值 |
|---|---|
| Group Name | `arm` |
| Kinematic Solver | `kdl_kinematics_plugin/KDLKinematicsPlugin` |
| Group Type | Chain |
| Base Link | `base_link` |
| Tip Link | `tool0` |

具体操作：

1. 点击 **Add Group**；
2. Group Name 填 `arm`；
3. Kinematic Solver 选择 KDL；
4. 点击 **Add Kin. Chain**；
5. Base Link 选择 `base_link`；
6. Tip Link 选择 `tool0`；
7. 保存 Group。

对于标准串联机械臂，优先使用 Chain。它不仅更容易满足 KDL 的串联链要求，也会把
固定法兰链接 `tool0` 纳入组的语义范围。

如果使用 **Add Joints** 只选择 `joint_1` 到 `joint_6`，活动自由度虽然正确，但
`tool0` 可能不属于 `arm`。这时再把 End Effector 的 Parent Link 设为 `tool0`，就会
产生“XML 合法、MoveIt 语义不完整”的 SRDF。

离开本页前必须确认：

```text
arm 的活动关节：joint_1 ... joint_6
arm 的末端链接：tool0
```

如果坚持使用 Add Joints，需要同时用 **Add Links** 把 `tool0` 加入 `arm`；
更简单的做法仍然是重新建立 `base_link → tool0` Chain。

注意：

- `virtual_joint` 不属于电机控制器；
- fixed joint 不产生可控自由度；
- KDL 只负责运动学，不负责路径搜索；
- IK 超时时间过小会导致交互标记经常变红。

![image-20261106140411365](assets/image-20261106140411365.png)

![image-20261106140441518](assets/image-20261106140441518.png)

### 4.6 配置夹爪 Planning Group

当前夹爪是平行两指结构。推荐新建：

| 字段 | 值 |
|---|---|
| Group Name | `gripper` |
| Kinematic Solver | None |
| 添加方式 | Add Links |
| Links | `gripper_base`、`left_finger`、`right_finger`、`gripper_tcp` |

夹爪通常不需要六维 IK，因此 solver 选择 None。

也可以使用 Add Joints 选择两个手指关节，但要记住 `right_finger_joint` 是 mimic joint，真正的命令变量只有 `left_finger_joint`。

当前仓库的 SRDF 已经包含 `arm`、`gripper`、`open`、`closed` 和
`gripper_eef`。其中真正接受控制命令的是 `left_finger_joint`，
`right_finger_joint` 继续由 URDF 的 mimic 关系驱动。

![image-20261106140709071](assets/image-20261106140709071.png)

![image-20261106140625137](assets/image-20261106140625137.png)

### 4.7 Robot Poses

建议至少建立：

| 名称 | 规划组 | 用途 |
|---|---|---|
| `home` | arm | 安全零位 |
| `ready` | arm | 常用准备位 |
| `open` | gripper | 夹爪打开 |
| `closed` | gripper | 夹爪闭合 |

创建姿态时逐个拖动关节，顺便检查关节限位是否合理。对于本模型：

- `left_finger_joint = 0.0`：闭合；
- `left_finger_joint = 0.025`：打开；
- 右指位置由 mimic 关系计算。

![image-20261106140742773](assets/image-20261106140742773.png)

![image-20261106140947058](assets/image-20261106140947058.png)

### 4.8 End Effectors

推荐：

| 字段 | 值 |
|---|---|
| End Effector Name | `gripper_eef` |
| End Effector Group | `gripper` |
| Parent Link | `tool0` |
| Parent Group | `arm` |

`tool0` 是机械臂法兰，`gripper` 是安装在法兰上的完整末端组。`gripper_tcp` 是抓取任务的参考点，不等同于 End Effector Group。

这是一个跨页面语义关系，Setup Assistant 不一定阻止你保存错误组合。点击保存前回到
Planning Groups 检查一次：

- `arm` 是否包含 Parent Link `tool0`；
- `gripper` 是否与 `arm` 分开，不要把手指关节放进机械臂控制器；
- `gripper_eef` 的 Parent Group 是否为 `arm`；
- `gripper_tcp` 只是任务坐标，不要误选成 End Effector Group。

如果后续日志出现下面内容，说明本页和 Planning Groups 页的组合不一致：

```text
Group 'arm' was specified as parent group for end-effector 'gripper_eef'
but it does not include the parent link 'tool0'
```

![image-20261106141029128](assets/image-20261106141029128.png)

### 4.9 Passive Joints

真正不可驱动、又不是 mimic 自动关系的关节才放进 Passive Joints。

当前 `right_finger_joint` 已经通过 URDF 的 `<mimic>` 表达，不要因为它没有电机就随意重复配置成 passive，除非你的驱动与模型策略明确要求这样做。

### 4.10 ros2_control 与控制器页面

Setup Assistant 可以生成：

- `<robot>.ros2_control.xacro`；
- `ros2_controllers.yaml`；
- `moveit_controllers.yaml`。

机械臂至少需要：

- 六个关节的 position command interface；
- position/velocity state interface；
- `joint_state_broadcaster`；
- `arm_controller`，类型为 `JointTrajectoryController`。

夹爪需要独立控制器。可以使用 `GripperActionController`，也可以像当前工程一样，
使用只包含 `left_finger_joint` 的 `JointTrajectoryController`。两种方案的 Action
类型不同，`ros2_controllers.yaml` 与 `moveit_controllers.yaml` 必须成对匹配。

当前工程在 Setup Assistant 中按以下方式操作：

1. **ros2_control URDF Modification** 页面选择 position command interface；
2. state interface 选择 position 和 velocity；
3. 六个机械臂关节都有 command/state interface；
4. 夹爪只给 `left_finger_joint` 添加 command/state interface；
5. 不给 mimic 的 `right_finger_joint` 添加独立 command interface；
6. **ROS 2 Controllers** 页面添加 `arm_controller`，类型选
   `JointTrajectoryController`，关节选择 `joint_1` 到 `joint_6`；
7. 添加 `gripper_controller`，类型同样选 `JointTrajectoryController`，只选择
   `left_finger_joint`。

#### GUI 能配置的内容与必须手工检查的内容

Setup Assistant 页面主要负责控制器名称、类型和关节集合。MoveIt Controller Manager
插件运行时需要的字段不一定都在 GUI 中暴露，因此生成后必须打开
`moveit_controllers.yaml` 检查：

| ros2_control 控制器 | MoveIt 类型 | 必需 `action_ns` | 当前关节集合 |
|---|---|---|---|
| `arm_controller` / `JointTrajectoryController` | `FollowJointTrajectory` | `follow_joint_trajectory` | `joint_1`～`joint_6` |
| `gripper_controller` / `JointTrajectoryController` | `FollowJointTrajectory` | `follow_joint_trajectory` | `left_finger_joint` |
| 可选 `GripperActionController` | `GripperCommand` | `gripper_cmd` | 主动夹爪关节 |

每个 MoveIt 控制器条目还建议明确写出 `default: true`。这些是插件专用字段，
不是 URDF 或 SRDF 的一部分；GUI 没有输入框时，不是让你跳过，而是要在生成后手工补齐。

![image-20261106141126381](assets/image-20261106141126381.png)

![image-20261106141155088](assets/image-20261106141155088.png)

![image-20261106141203303](assets/image-20261106141203303.png)

### 4.11 生成包

输出路径选择：

```text
~/open_source_project/robot_arm_moveit/src/robot_arm_moveit_config
```

首次生成可以直接使用该路径。编辑现有配置包并再次生成时，先保存当前差异：

```bash
git diff -- src/robot_arm_moveit_config
```

Setup Assistant 会重新写入多个 YAML、Xacro、SRDF 和 RViz 文件，可能覆盖之前的人工修复。
生成后立即再次运行 `git diff`，逐个确认变化，不要把“工具生成成功”理解为“配置已经验收”。

生成后重新构建：

```bash
cd ~/open_source_project/robot_arm_moveit
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select robot_arm_description robot_arm_moveit_config \
  --symlink-install
source install/setup.bash
```

![image-20261106141248798](assets/image-20261106141248798.png)

**![image-20261106141256686](assets/image-20261106141256686.png)**

![image-20261106141307748](assets/image-20261106141307748.png)

![image-20261106141402659](assets/image-20261106141402659.png)

![image-20261106141418742](assets/image-20261106141418742.png)

### 4.12 生成文件速查

| 文件 | 作用 |
|---|---|
| `.setup_assistant` | Setup Assistant 再次编辑时使用的元数据 |
| `*.srdf` | 规划组、虚拟关节、姿态、末端、自碰撞矩阵 |
| `kinematics.yaml` | 每个规划组的 IK 插件 |
| `joint_limits.yaml` | MoveIt 侧速度和加速度限制 |
| `moveit_controllers.yaml` | MoveIt 到 Action 控制器的映射 |
| `ros2_controllers.yaml` | ros2_control Controller Manager 配置 |
| `*.ros2_control.xacro` | 硬件插件及 command/state interface |
| `initial_positions.yaml` | Mock Hardware 初始位置 |
| `moveit.rviz` | MotionPlanning 面板布局 |
| `demo.launch.py` | RViz + MoveGroup + Mock Hardware 一键启动 |

### 4.13 生成后必须人工检查（前面检查点的最终汇总）

Setup Assistant 生成成功，不代表运行时一定正确。尤其是再次点击生成后，工具可能会
覆盖之前人工修复过的 YAML、Xacro 和 SRDF。每次重新生成都应重新检查以下项目。

#### 检查 1：YAML 数字类型

src/robot_arm_moveit_config/config/joint_limits.yaml

错误写法：

```yaml
max_velocity: 2
max_acceleration: 0
```

ROS 2 会把它们识别成 integer，而 MoveIt 声明的是 double，启动时会报：

```text
expected [double] got [integer]
```

正确写法：

```yaml
max_velocity: 2.0
max_acceleration: 0.0
```

不要只修改日志点名的第一个关节。参数加载按顺序进行，修完 `joint_2` 后，下一次可能才会
报 `joint_3` 或夹爪关节。必须检查 `joint_limits` 下的**全部关节**。

可以用下面的脚本一次检查所有速度和加速度值：

```bash
python3 - <<'PY'
import pathlib
import yaml

path = pathlib.Path(
    "src/robot_arm_moveit_config/config/joint_limits.yaml")
limits = yaml.safe_load(path.read_text())["joint_limits"]
for joint, values in limits.items():
    for key in ("max_velocity", "max_acceleration"):
        value = values.get(key)
        assert isinstance(value, float), (
            f"{joint}.{key} 不是 double：{value!r}")
print("所有 joint limit 数值均为 double")
PY
```

#### 检查 2：控制器 Action 命名空间

`moveit_controllers.yaml` 应包含：

```yaml
moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6

  gripper_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - left_finger_joint
```

缺少 `action_ns` 时，MoveIt 不能连接对应 Action。机械臂和夹爪是两个独立条目，
不能只修 `arm_controller` 而漏掉 `gripper_controller`。健康日志应包含：

```text
Added FollowJointTrajectory controller for arm_controller
Added FollowJointTrajectory controller for gripper_controller
Returned 2 controllers in list
```

#### 检查 3：Xacro API 是否被重新生成覆盖

Humble 中应使用：

```xml
${xacro.load_yaml(initial_positions_file)}
```

如果重新生成后变回 `${load_yaml(...)}`，虽然通常仍可运行，但会持续出现弃用警告，
应恢复为带 `xacro.` 前缀的写法。

#### 检查 4：末端执行器父链接是否属于父规划组

如果 SRDF 写了：

```xml
<end_effector name="gripper_eef"
              parent_link="tool0"
              group="gripper"
              parent_group="arm"/>
```

那么 `arm` 组必须包含 `tool0`。如果机械臂组是用活动关节逐个添加的，需要额外加入：

```xml
<group name="arm">
  <!-- joint_1 ... joint_6 -->
  <link name="tool0"/>
</group>
```

否则会出现 `parent group ... does not include the parent link 'tool0'`。更推荐在
Setup Assistant 中把机械臂定义为从 `base_link` 到 `tool0` 的 Chain，可以自然包含法兰链接。

#### 检查 5：重新生成后的完整验收

```bash
colcon build \
  --packages-select robot_arm_description robot_arm_moveit_config \
  --symlink-install
source install/setup.bash

ros2 launch robot_arm_moveit_config demo.launch.py

# 另开终端检查
ros2 control list_controllers
ros2 action list -t | grep follow_joint_trajectory
```

当前工程正常时应有 3 个 active 控制器，并出现机械臂和夹爪两条轨迹 Action。

---

## 5. 补全 ros2_control 与夹爪控制

### 5.1 三层配置不要混淆

| 层 | 文件 | 回答的问题 |
|---|---|---|
| 硬件接口层 | `*.ros2_control.xacro` | 哪些关节能接收命令、能反馈什么 |
| 控制器层 | `ros2_controllers.yaml` | 哪个控制器占用哪些接口 |
| MoveIt 映射层 | `moveit_controllers.yaml` | MoveIt 应向哪个 Action 发轨迹 |

ros2_control 的整体机制见 [Getting Started](https://control.ros.org/humble/doc/getting_started/getting_started.html)。

### 5.2 机械臂 ros2_control 接口

Mock Hardware 示例：

```xml
<ros2_control name="RobotSystem" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
  </hardware>

  <joint name="joint_1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- joint_2 ... joint_6 同理 -->
</ros2_control>
```

### 5.3 为夹爪增加接口

当前工程只把主动关节导出给 ros2_control：

```xml
<joint name="left_finger_joint">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

mimic 关系保留在原始 URDF 中，MoveIt 和 RobotState 会根据 `left_finger_joint`
计算右指位置。如果实机确实有右指编码器，可以为 `right_finger_joint` 导出 state
interface，但不要再给它 position command interface，否则两个命令源可能互相冲突。

### 5.4 当前方案：JointTrajectoryController

Setup Assistant 为当前工程生成的是单关节轨迹控制器：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController

gripper_controller:
  ros__parameters:
    joints:
      - left_finger_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

该方案使用 `FollowJointTrajectory` Action，优点是与机械臂轨迹控制器一致，
也便于指定夹爪运动时间。只控制主动的 `left_finger_joint`，不要把 mimic 关节加入控制器。

确保启动时也生成一个 gripper spawner：

```bash
ros2 run controller_manager spawner gripper_controller
```

`GripperActionController` 也是有效的替代方案，适合需要 `max_effort`、堵转检测和
`GripperCommand` 语义的实机夹爪。切换方案时必须同时修改 ros2_control 和 MoveIt
两侧配置，不能只修改其中一个文件。参数参考
[Gripper Action Controller](https://control.ros.org/humble/doc/ros2_controllers/gripper_controllers/doc/userdoc.html)。

### 5.5 告诉 MoveIt 如何控制夹爪

`moveit_controllers.yaml` 示例：

```yaml
moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6

  gripper_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - left_finger_joint
```

两侧必须满足以下对应关系：

| ros2_control 控制器 | MoveIt `type` | `action_ns` |
|---|---|---|
| `JointTrajectoryController` | `FollowJointTrajectory` | `follow_joint_trajectory` |
| `GripperActionController` | `GripperCommand` | `gripper_cmd` |

MoveIt 参数说明见 [Low Level Controllers](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html)。

### 5.6 单独测试夹爪

```bash
ros2 control list_controllers
ros2 action list -t | grep gripper

ros2 action send_goal \
  /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [left_finger_joint], points: [{positions: [0.02], time_from_start: {sec: 1}}]}}"
```

成功时会返回 `Goal successfully reached!`，随后 `/joint_states` 中
`left_finger_joint` 应接近 `0.02`。

本模型中 0.0 接近闭合，0.025 接近全开。第一次接实机时不要直接使用模型最大值，应先在低速度、低力矩下标定安全范围。

---

## 6. 运行 Demo 并完成第一次规划

### 6.1 启动

```bash
cd ~/open_source_project/robot_arm_moveit
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_moveit_config demo.launch.py
```

健康启动通常会出现：

```text
Added FollowJointTrajectory controller for arm_controller
Added FollowJointTrajectory controller for gripper_controller
Returned 2 controllers in list
You can start planning now!
Ready to take commands for planning group arm.
```

![RViz MoveIt Demo](assets/rviz_moveit_demo.png)

### 6.2 在 RViz 中规划

在 MotionPlanning 面板中：

1. **Planning Group** 选择 `arm`；
2. **Start State** 选择 Current；
3. **Goal State** 可先选择命名姿态 `initial` 或 `home`；
4. 点击 **Plan**，只计算并显示轨迹；
5. 确认轨迹没有穿过机器人或环境；
6. 点击 **Execute** 执行已经规划的轨迹；
7. 熟悉后再使用 **Plan & Execute**。

使用交互标记时：

- 红色通常表示 IK 无解或目标碰撞；
- 绿色表示存在可用 IK；
- 交互标记消失时，检查 Planning Group 和 IK 配置；
- 目标离机器人太远时，不要先怀疑规划器，先检查工作空间和模型尺寸。

### 6.3 Plan 与 Execute 为什么要分开

实机调试阶段推荐始终分两步：

1. Plan：检查目标、碰撞和轨迹；
2. Execute：经过人工或程序安全校验后执行。

`Plan & Execute` 适合成熟流程，不适合第一次接通真实电机。

### 6.4 当前 Demo 的运行拓扑

下面是本工程 Demo 运行时由 `rqt_graph` 采集的真实节点图：

![MoveIt Demo 节点图](assets/moveit_node_graph.png)

关键节点和接口：

| 名称 | 作用 |
|---|---|
| `/move_group` | 规划、场景、执行总入口 |
| `/robot_state_publisher` | 根据 URDF 和 joint_states 发布 TF |
| `/controller_manager` | 管理硬件和控制器 |
| `/arm_controller/follow_joint_trajectory` | 六轴轨迹 Action |
| `/gripper_controller/follow_joint_trajectory` | 夹爪轨迹 Action |
| `/joint_state_broadcaster` | 发布 `/joint_states` |
| `/monitored_planning_scene` | MoveIt 规划场景同步 |
| `/tf`、`/tf_static` | 坐标变换 |

### 6.5 常用验收命令

另开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/open_source_project/robot_arm_moveit/install/setup.bash

ros2 node list
ros2 control list_controllers
ros2 action list -t
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo world tool0
```

期望控制器状态：

```text
arm_controller             active
gripper_controller         active
joint_state_broadcaster    active
```

---

## 7. 用 C++ 调用 MoveIt

官方入门参考：[Your First C++ MoveIt Project](https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html) 和 [MoveGroupInterface](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html)。

下面给出一个最小的关节目标规划与执行示例。

```cpp
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "robot_arm_moveit_client",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setPlanningTime(5.0);

  std::vector<double> target = {
    0.30, -0.60, 0.90, 0.20, -0.40, 0.00
  };
  move_group.setJointValueTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned =
    static_cast<bool>(move_group.plan(plan));

  if (planned) {
    const auto result = move_group.execute(plan);
    if (!result) {
      RCLCPP_ERROR(node->get_logger(), "轨迹执行失败");
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "轨迹规划失败");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
```

`CMakeLists.txt` 关键部分：

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(move_arm src/move_arm.cpp)
ament_target_dependencies(
  move_arm
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS move_arm
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

执行前必须已经启动 `demo.launch.py` 或真实 bringup：

```bash
ros2 run your_package move_arm
```

实机代码还应增加：

- 目标值范围检查；
- 当前状态时间戳检查；
- 规划结果碰撞复核；
- 控制器在线检查；
- 超时和取消；
- 急停状态检查；
- 执行失败后的安全回退。

---

## 8. 规划场景、抓取与感知

### 8.1 PlanningScene 是什么

PlanningScene 同时保存：

- 当前 RobotState；
- 世界中的碰撞物；
- 附着在机器人上的物体；
- Allowed Collision Matrix；
- 可选的 OctoMap。

PlanningSceneMonitor 会通过 `/joint_states`、TF、碰撞物话题和传感器持续更新它。详见 [Planning Scene Monitor](https://moveit.picknik.ai/humble/doc/concepts/planning_scene_monitor.html)。

### 8.2 抓取任务的典型状态机

```text
检测目标
  → 添加 CollisionObject
  → 规划到预抓取位
  → 张开夹爪
  → 直线接近
  → 闭合夹爪
  → attachObject
  → 抬升
  → 搬运
  → 放置
  → detachObject
  → 张开夹爪
  → 返回安全位
```

关键点：

- 抓住后必须 attach，规划器才知道物体会随机器人移动；
- attach 时要设置 `touch_links`，允许物体与手指接触；
- 放下后先 detach，再更新世界中的物体位姿；
- 接近和撤离常用笛卡尔路径；
- 不要在实机上关闭 Cartesian jump threshold 却不做额外安全检查。

官方示例见 [Planning Scene ROS API](https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)。

### 8.3 Gazebo 与 MoveIt 联调效果

![Gazebo 与 MoveIt 示例](assets/gazebo_moveit_example.png)

仿真阶段应验证：

- Gazebo 关节方向与 URDF 一致；
- `/joint_states` 来自仿真而不是重复的假节点；
- MoveIt 的碰撞模型与 Gazebo 外形对齐；
- 控制器 Action 名称一致；
- 抓取时物理附着与 PlanningScene attach 同步；
- 仿真能稳定完成多次抓放循环。

### 8.4 三维感知

深度相机或点云通常通过 OccupancyMapMonitor 更新 OctoMap。未配置传感器时看到：

```text
No 3D sensor plugin(s) defined for octomap updates
```

不影响纯模型碰撞规划。需要感知时再配置 `sensors_3d.yaml`、TF 和点云/深度图话题。

---

## 9. 从 Mock Hardware 切换到实机

### 9.1 总体原则

只要关节名、接口和 Action 契约保持不变，MoveIt 配置大部分可以复用。真正需要替换的是：

```xml
<plugin>mock_components/GenericSystem</plugin>
```

将它换成自己的硬件插件，例如：

```xml
<plugin>robot_arm_hardware/RobotArmSystem</plugin>
<param name="device">can0</param>
<param name="control_rate">500</param>
```

![实机控制闭环](assets/real_robot_control_loop.svg)

### 9.2 创建硬件插件包

```bash
cd ~/open_source_project/robot_arm_moveit/src

ros2 pkg create robot_arm_hardware \
  --build-type ament_cmake \
  --dependencies \
    hardware_interface \
    pluginlib \
    rclcpp \
    rclcpp_lifecycle
```

多关节机械臂通常继承：

```cpp
hardware_interface::SystemInterface
```

核心方法：

| 方法 | 作用 |
|---|---|
| `on_init()` | 读取 URDF 硬件参数、建立内部数据结构 |
| `on_configure()` | 打开 CAN/EtherCAT/TCP 等通信 |
| `export_state_interfaces()` | 导出位置、速度、力矩反馈 |
| `export_command_interfaces()` | 导出位置、速度或力矩命令 |
| `on_activate()` | 使能设备、同步初始命令 |
| `read()` | 从驱动器读取编码器和状态 |
| `write()` | 向驱动器发送控制命令 |
| `on_deactivate()` | 停止运动、撤销使能 |
| `on_error()` | 执行故障安全处理 |

官方完整步骤见 [Writing a Hardware Component](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)。

### 9.3 插件导出

源文件末尾：

```cpp
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  robot_arm_hardware::RobotArmSystem,
  hardware_interface::SystemInterface)
```

插件 XML：

```xml
<library path="robot_arm_hardware">
  <class
    name="robot_arm_hardware/RobotArmSystem"
    type="robot_arm_hardware::RobotArmSystem"
    base_class_type="hardware_interface::SystemInterface">
    <description>Six-axis robot arm hardware interface</description>
  </class>
</library>
```

CMake 中导出：

```cmake
pluginlib_export_plugin_description_file(
  hardware_interface
  robot_arm_hardware.xml
)
```

### 9.4 read-update-write 闭环

Controller Manager 周期执行：

```text
read() → controller update() → write()
```

- `read()` 把编码器转换为弧度和 rad/s；
- controller 根据目标轨迹计算本周期命令；
- `write()` 把命令转换成驱动器协议单位；
- `joint_state_broadcaster` 发布真实状态；
- MoveIt 使用真实状态作为下一次规划起点。

必须明确：

- 编码器零点；
- 减速比；
- 正负方向；
- 编码器脉冲到弧度的换算；
- 电机侧与关节侧单位；
- 通信超时；
- 驱动器故障码；
- 抱闸释放时序。

### 9.5 实机安全底线

> MoveIt 的碰撞检测不是安全 PLC，软件限位也不能替代机械限位和急停。

首次联动前至少具备：

- [ ] 独立硬件急停；
- [ ] 驱动器硬限位或安全限位；
- [ ] 低速模式；
- [ ] 低电流/低力矩模式；
- [ ] 通信看门狗；
- [ ] 轨迹跟随误差阈值；
- [ ] 指令超时自动停止；
- [ ] 控制器失活时保持或制动；
- [ ] 人员远离运动范围；
- [ ] 上电姿态与 ROS 当前状态一致。

### 9.6 推荐实机验收阶梯

不要一步到位执行 MoveIt 轨迹。按顺序验收：

1. **只读状态**：不使能电机，只验证编码器和 `/joint_states`；
2. **单轴点动**：低速、短距离验证每个关节正方向；
3. **限位验证**：软件限位、驱动器限位、机械限位一致；
4. **多轴控制器**：让 JointTrajectoryController 接受小幅轨迹；
5. **MoveIt 只规划**：Plan，不 Execute；
6. **低速执行**：缩放系数 0.05～0.1；
7. **碰撞物验证**：加入工作台、围栏和工具；
8. **夹爪验证**：先空载，再软物体，再真实工件；
9. **异常测试**：断通信、急停、驱动故障、取消轨迹；
10. **生产节拍**：安全验证通过后再逐步提高速度。

---

## 10. 常见故障与定位方法

### 10.1 故障矩阵

| 现象/日志 | 根因 | 处理 |
|---|---|---|
| `PackageNotFoundError: robot_arm_description` | description 不是 ament 包，或未构建/source | 使用 `ament_cmake`、调用 `ament_package()`，重新构建并 source |
| Setup Assistant 选路径后卡住 | XML/URDF 错误，或 `package://` 资源所属包不可见 | 先运行 `check_urdf`，再检查 description 包与 `ros2 pkg prefix` |
| `expected [double] got [integer]` | YAML 中 `2`、`0` 被解析为 integer | 改成 `2.0`、`0.0` |
| 修完一个关节后又报下一个关节类型错误 | 只修了日志首先加载到的字段 | 遍历检查所有关节的 `max_velocity` 和 `max_acceleration` |
| `No action namespace specified` | `moveit_controllers.yaml` 缺 `action_ns` | 添加 `follow_joint_trajectory` 或 `gripper_cmd` |
| `Returned 0 controllers in list` | MoveIt 控制器映射错误 | 检查 controller name、type、action_ns、joints |
| `parent group ... does not include ... tool0` | End Effector 的父链接不属于 `arm` 组 | 使用 `base_link → tool0` Chain，或把 `tool0` link 加入 `arm` |
| `Using load_yaml() directly is deprecated` | 重新生成后 Xacro 使用旧 API | 改为 `xacro.load_yaml()` |
| 重新生成后旧错误复现 | Setup Assistant 覆盖了人工修复 | 每次生成后重新执行 4.13 节验收清单 |
| 选择 `gripper` 时提示 `No active joints or end effectors found` | 一维夹爪没有 IK/六维交互标记 | 若命名姿态、Action 和控制器正常，可忽略该 RViz 交互提示 |
| `robot_description_semantic` 超时 | `move_group` 先崩溃或 SRDF 没传给 RViz | 先找 move_group 最早的异常，不要只修 RViz |
| `No 3D sensor plugin` | 没配置 OctoMap 传感器 | 无感知需求可忽略；有需求配置 `sensors_3d.yaml` |
| `root link has an inertia` | KDL 不支持根 link 惯量 | 可增加无惯量 dummy root；不做动力学时通常不致命 |
| `/recognize_objects not available` | 没启动物体识别 Action | 无识别需求可忽略 |
| `Could not enable FIFO RT scheduling` | 普通用户无实时调度权限 | 仿真可忽略；实机按 ros2_control 文档配置 realtime 权限 |
| `Controller not active` | spawner 失败或接口冲突 | `ros2 control list_controllers`、检查 command interface |
| `Failed to fetch current robot state` | `/joint_states` 不更新或时间戳错误 | 检查频率、ROS time、关节名 |
| 交互标记为红色 | IK 无解、目标碰撞或超出限位 | 拉近目标、检查 solver、碰撞和 limits |
| Plan 成功但 Execute 失败 | Action 名、控制器状态或关节集合不一致 | 检查 `ros2 action list -t` 和控制器 YAML |
| 退出时出现 `-11` | Humble 某些插件卸载顺序问题 | 若只在 Ctrl-C 后发生，和运行中崩溃分开判断 |

### 10.2 从“最早的错误”开始看

一个典型连锁故障：

```text
joint_limits 类型错误
  → move_group 退出
  → robot_description_semantic 不再发布
  → RViz SRDF 加载失败
  → MotionPlanning 面板不可用
```

真正根因是第一条，而不是最后一条。排查原则：

1. 找第一个退出的进程；
2. 找它退出前的第一条 exception；
3. 不要被后续十几条连锁错误带偏。

### 10.3 一组实用诊断命令

```bash
# 包与环境
ros2 pkg prefix robot_arm_description
ros2 pkg prefix robot_arm_moveit_config
echo $AMENT_PREFIX_PATH | tr ':' '\n' | head

# 模型
check_urdf src/robot_arm_description/urdf/generic_6dof_arm_with_gripper.urdf
xacro src/robot_arm_moveit_config/config/generic_6dof_arm_with_gripper.urdf.xacro \
  initial_positions_file:=$PWD/src/robot_arm_moveit_config/config/initial_positions.yaml \
  > /tmp/generated_robot.urdf
check_urdf /tmp/generated_robot.urdf

# 控制
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 action list -t
ros2 topic hz /joint_states

# TF
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo base_link tool0

# MoveIt
ros2 node info /move_group
ros2 topic echo /monitored_planning_scene --once
```

---

## 11. 推荐练习路线

### 练习 1：理解模型

- 修改 `link_2` 的长度；
- 同步修改 joint origin；
- 运行 `check_urdf`；
- 重新打开 Setup Assistant；
- 观察运动树和 RViz 外形变化。

验收：末端位置变化符合预期，没有断链或自碰撞异常。

### 练习 2：完善夹爪

- 添加 `gripper` Planning Group；
- 添加 `open`、`closed` 姿态；
- 添加 End Effector；
- 补齐 ros2_control 手指接口；
- 启动 `gripper_controller`；
- 用 Action 命令开合夹爪。

验收：左指受命令，右指通过 mimic 同步，MoveIt 能识别末端组。

### 练习 3：编程规划

- 编写 MoveGroupInterface 节点；
- 支持关节目标和位姿目标；
- 把速度缩放设为 0.1；
- 增加规划失败处理；
- 增加取消执行功能。

验收：连续执行 20 次不崩溃，失败时不会发送旧轨迹。

### 练习 4：规划场景

- 添加桌面 CollisionObject；
- 添加一个待抓取方块；
- 完成 attach/detach；
- 验证附着后机械臂不会让工件撞桌面。

验收：PlanningScene 与仿真世界一致。

### 练习 5：实机驱动

- 先实现只读 HardwareInterface；
- 再实现单轴低速写入；
- 最后接入 JointTrajectoryController；
- 完成急停、断网和跟随误差测试。

验收：任何通信或驱动故障都能进入可预测的安全状态。

---

## 12. 术语与参考资料

### 12.1 术语速查

| 术语 | 含义 |
|---|---|
| FK | 已知关节角，计算末端位姿 |
| IK | 已知末端位姿，求关节角 |
| TCP | Tool Center Point，工具中心点 |
| DOF | Degree of Freedom，自由度 |
| SRDF | MoveIt 的语义机器人描述 |
| ACM | Allowed Collision Matrix，允许碰撞矩阵 |
| PSM | PlanningSceneMonitor |
| OMPL | MoveIt 默认集成的采样规划库 |
| JTC | JointTrajectoryController |
| FJT | FollowJointTrajectory Action |
| mimic joint | 位置由另一个关节线性决定的从动关节 |
| HardwareInterface | ros2_control 对仿真/实机硬件的插件抽象 |

### 12.2 官方资料

- [MoveIt 2 Humble Concepts](https://moveit.picknik.ai/humble/doc/concepts/concepts.html)
- [MoveIt Setup Assistant](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)
- [URDF and SRDF](https://moveit.picknik.ai/humble/doc/examples/urdf_srdf/urdf_srdf_tutorial.html)
- [Motion Planning](https://moveit.picknik.ai/humble/doc/concepts/motion_planning.html)
- [Planning Scene Monitor](https://moveit.picknik.ai/humble/doc/concepts/planning_scene_monitor.html)
- [MoveGroupInterface Tutorial](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html)
- [MoveIt Controller Configuration](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html)
- [ros2_control Getting Started](https://control.ros.org/humble/doc/getting_started/getting_started.html)
- [Writing a Hardware Component](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
- [ros2_control 6-DOF Tutorial](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html)
- [Mimic Joint Kinematics](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/joints_userdoc.html)
- [Gripper Action Controller](https://control.ros.org/humble/doc/ros2_controllers/gripper_controllers/doc/userdoc.html)

---

## 结语

学习 MoveIt 2 最有效的方法，不是记住所有参数，而是始终抓住这条主线：

```text
模型正确
  → 语义配置正确
  → 当前状态可信
  → 碰撞环境可信
  → 规划得到轨迹
  → 控制器接受轨迹
  → 硬件反馈闭环
  → 独立安全链兜底
```

当系统报错时，也沿这条链从前往后检查。只要模型、状态、规划、控制和硬件五层边界清楚，大多数 MoveIt 问题都能快速定位。

# reBot Arm ROS2 SDK

<p align="center">
  <img src="./media/rebot_arm_b601.png" alt="reBot Arm" width="720">
</p>

<p align="center">
  <strong>ROS2 · 机械臂控制 · 夹爪控制 · 轨迹接口 · RViz 可视化 · 全开源</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble | Jazzy-blue.svg" alt="ROS2 Humble">
  <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Python 3.10">
  <img src="https://img.shields.io/badge/Version-v0.3.0-brightgreen.svg" alt="Version v0.3.0">
  <img src="https://img.shields.io/badge/Platform-Ubuntu%2022.04+-orange.svg" alt="Ubuntu 22.04+">
  <img src="https://img.shields.io/badge/Hardware-DM%20%7C%20RS-lightgrey.svg" alt="DM and RS">
</p>

<p align="center">
  <strong>
    <a href="./README_zh.md">简体中文</a> &nbsp;|&nbsp;
    <a href="./README.md">English</a> &nbsp;|&nbsp;
    <a href="./API_zh.md">API 文档</a>
  </strong>
</p>

**启动操作入口：** [各功能启动操作速查](#各功能启动操作速查)（仿真、真机、手势、手掌跟随、本地语音、RViz 字幕、MoveIt、MuJoCo、Agent）。

---

## 关于本仓库（二次开发版）

本仓库基于 Seeed 官方开源项目 [reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2)
修改，供个人学习与交流使用。在原版基础上新增/修改了：

| 新增内容 | 说明 |
|---|---|
| `rebotarm_greeting` | 打招呼演示：三位置点头，topic 触发（见[演示项目](#演示项目)） |
| `rebotarm_dance` | 跳舞演示：约 38s 舞蹈，轨迹离线预计算（见[演示项目](#演示项目)） |
| `rebotarm_vision` | 手势识别和手掌位置跟随：摄像头 + MediaPipe |
| `rebotarm_voice` | 本地免费 Whisper 语音控制与 RViz 中文字幕，无需 API Key |
| `rebotarm_gesture_actions` | 手势触发的 TCP 轨迹动作（石头 / 布 / 比心） |
| `fake_dm_bringup.launch.py` | DM 版仿真启动（无真机也可运行） |

### 安装方式

1. **环境配置**：参照官方 wiki 文档，完成步骤 1、2、4（安装 ROS2、依赖、motorbridge）。
   官方文档的步骤 3、5（两次 git clone）**不用单独执行**，已合并到下方第 2 条的命令中：
   <https://wiki.seeedstudio.com/cn/rebot_arm_b601_dm_ros2_integration/>
2. **克隆本仓库和底层 SDK**（对应官方步骤 3 和 5。注意：工作空间用本仓库地址，
   底层 SDK `reBotArm_control_py` 仍用 Seeed 官方地址，本仓库不包含 third_party）：

   ```bash
   git clone https://github.com/haotian34/rebotarm_ros2_breakice.git
   cd rebotarm_ros2_breakice
   mkdir -p third_party
   git clone https://github.com/Seeed-Projects/reBotArm_control_py.git third_party/reBotArm_control_py
   ```

3. **编译运行**：

   ```bash
   source /opt/ros/jazzy/setup.bash   # Ubuntu 22.04 请换成 humble
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch rebotarm_bringup fake_dm_bringup.launch.py
   ```

4. **手势识别依赖**（可选，仅视觉演示需要）：

   ```bash
   pip install --user --break-system-packages "mediapipe==0.10.14" "numpy==1.26.4"
   ```

   `numpy` 必须保持 <2，否则会破坏 ROS2/pinocchio。

其余使用方法与官方版本一致，详见下文。

---

## 项目介绍

当前版本：`v0.3.0`

`rebotarm_ros2` 是 reBot Arm B601 DM 和 RS 机械臂的 ROS2 SDK 工作空间。它将新版
`reBotArm_control_py` Python 控制库封装为 ROS2 topic、service 和 action，
作为二次开发、上层规划、可视化、重力补偿和单电机调试的统一入口。

当前工作空间包含五个核心 ROS2 包：

| 包 | 作用 |
|---|---|
| `rebotarm_msgs` | 自定义 msg / srv / action 接口 |
| `rebotarmcontroller` | 控制节点包，提供 `reBotArmController` 节点 |
| `rebotarm_bringup` | launch、配置、URDF、RViz 等启动资源 |
| `rebotarm_moveit_config` | MoveIt 2 配置、SRDF、ros2_control、RViz 配置 |
| `rebotarm_moveit_demos` | MoveIt 2 应用示例，如抓取放置和矩形轨迹 |

以及演示和上层应用包（见[启动操作速查](#各功能启动操作速查)）：

| 包 | 作用 |
|---|---|
| `rebotarm_greeting` | topic 触发的三位置点头问候 |
| `rebotarm_dance` | 舞蹈表演，关节轨迹离线预计算 |
| `rebotarm_vision` | 基于 MediaPipe 的手势识别（石头 / 布 / 比心） |
| `rebotarm_gesture_actions` | 手势到轨迹的分发与回放 |
| `rebotarm_voice` | 本地 Whisper 语音识别、动作控制与 RViz 中文字幕 |
| `rebotarm_mujoco_rs` | RS MuJoCo 场景、相机和任务接口 |
| `rebotarm_agent` | MCP 工具、文本 Agent 与 Dashboard |

---

## 核心功能

- 发布机械臂状态：`/rebotarm/joint_states`、`/rebotarm/arm_status`
- 提供基础服务：`enable`、`disable`、`set_zero`、`safe_home`
- 支持笛卡尔目标：`MoveToPoseIK` service、`MoveToPose` action
- 支持标准轨迹接口：`control_msgs/action/FollowJointTrajectory`
- 支持夹爪控制：`SetGripper` service、`GripperCommand` action
- 支持单个关节指令：`JointMitCmd`、`JointPosVelCmd`

---

## 环境配置

| 组件 | 型号 / 要求 |
|---|---|
| 机械臂 | reBot Arm B601 DM 或 RS |
| 通信接口 | DM：USB 串口桥；RS：SocketCAN |
| 主机 | Ubuntu 22.04+，ROS2，Python 3.10+ |
| 摄像头 | USB 摄像头（仅手势识别演示需要） |

接线说明：

1. 将 USB2CAN 串口桥接器连接到机械臂 CAN 总线。
2. 将夹爪电机接入同一条 CAN 总线。
3. 将 USB2CAN 插入主机，并确认设备名：

```bash
ls /dev/ttyACM*
```

如果需要临时开放串口权限：

```bash
sudo chmod 666 /dev/ttyACM0
```

RS 默认使用 SocketCAN。启动 ROS2 驱动前先拉起 CAN 接口：

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

---

## 配置开发环境

### Step 1. 安装 ROS2 依赖

请参考[ROS官方下载文档](https://www.ros.org/blog/getting-started/)选择适合的版本进行安装。

### Step 2. 获取 ROS2 源码

优先使用 Seeed-Projects 官方仓库：

```bash
git clone https://github.com/Seeed-Projects/reBotArmController_ROS2.git rebotarm_ros2
cd rebotarm_ros2
```

也可以使用当前开发仓库：

```bash
git clone https://github.com/EclipseaHime017/reBotArmController_ROS2.git rebotarm_ros2
cd rebotarm_ros2
```

### Step 3. 安装 motorbridge

`motorbridge` 从 PyPI 官方源安装：

```bash
python3 -m pip install --user --index-url https://pypi.org/simple motorbridge
```

### Step 4. 获取底层 SDK


```bash
mkdir -p third_party
git clone https://github.com/vectorBH6/reBotArm_control_py.git third_party/reBotArm_control_py
```

## 构建工作空间

```bash
colcon build --symlink-install
source install/setup.bash
```

验证包和入口：

```bash
ros2 pkg executables rebotarmcontroller
```

期望输出：

```text
rebotarmcontroller reBotArmController
rebotarmcontroller GravityCompensation
rebotarmcontroller GripperControl
rebotarmcontroller MoveTo
rebotarmcontroller MoveToPose
```

---

## 目录结构

```text
rebotarm_ros2/
├── README_zh.md
├── API_zh.md
├── PLAN.md
├── instruction.md
└── src/
    ├── rebotarm_msgs/
    │   ├── msg/
    │   ├── srv/
    │   └── action/
    ├── rebotarmcontroller/
    │   ├── rebotarmcontroller/
    │   │   ├── rebotarm_controller.py
    │   │   ├── hardware_manager.py
    │   │   ├── ros_publishers.py
    │   │   ├── ros_services.py
    │   │   ├── ros_actions.py
    │   │   ├── motor_passthrough.py
    │   │   ├── conversions.py
    │   │   └── examples/
    ├── rebotarm_bringup/
    │   ├── launch/
    │   ├── config/
    │   ├── description/
    │   └── rviz/
    ├── rebotarm_moveit_config/
    │   ├── config/
    │   └── launch/
    ├── rebotarm_moveit_demos/
    │   ├── config/
    │   ├── launch/
    │   └── rebotarm_moveit_demos/
    ├── rebotarm_greeting/
    │   ├── config/
    │   ├── launch/
    │   └── rebotarm_greeting/
    ├── rebotarm_dance/
    │   ├── config/
    │   ├── launch/
    │   └── rebotarm_dance/
    ├── rebotarm_gesture_actions/
    │   ├── config/
    │   ├── launch/
    │   └── rebotarm_gesture_actions/
    └── rebotarm_vision/
        ├── launch/
        ├── scripts/
        └── rebotarm_vision/
```

---

## 快速启动

在正式开始使用机械臂前请注意： **机械臂的控制器具有较高自由度，启用控制器或者给机械臂上电前务必注意械臂工作空间内无人和障碍物。同时，请严格审查每一次对机械臂的运动控制，避免出现意外。严禁危险操作，造成后果自负。**

### 启动完整系统

启动控制节点、`robot_state_publisher`，可选 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py
```

`reBotArmController` 启动时会直接连接真实硬件。默认型号由
`rebotarm_bringup/config/rebotarm_hardware.yaml` 中的 `default_model` 决定，
当前为 `dm`。

注意，如果使用 RS 版本的机械臂且未修改`default_model`时，需要显式传入 `model:=rs`，并指定 `channel:=can0`。
```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:={/dev/实际的串口名称}
ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can0
```

如果需要在 RViz 中可视化机械臂运动，可以启用 RViz：

```bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
```

### 只启动控制节点

和上述```bringup.launch.py```不同，如果希望最小化机械臂控制操作而不启用多余的`robot_state_publisher`
或者 RViz 可视化，那么可以直接启动：

```bash
ros2 launch rebotarm_bringup driver.launch.py
```

### 直接运行控制节点

```bash
ros2 run rebotarmcontroller reBotArmController
```

需要注意的是，和```driver.launch.py```从```rebotarm_bringup/config```不同，直接运行控制节点会从 SDK 加载
默认机械臂参数，因此更推荐通过 ros launch 的方式去启动节点。

---

## 直接移动到 Pose

不运行 demo 时，可以直接调用 ROS service 和 action 完成一次末端位姿移动。
先在一个终端启动控制节点：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

然后在另一个终端执行控制命令：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
```

1. 使能机械臂：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
```

2. 移动末端到目标 pose：

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, duration: 2.0}"
```

`move_to_pose` action 通过 SDK 末端控制器执行。机械臂控制模式由
`rebotarm_hardware.yaml` 决定：DM 默认 `posvel`，RS 默认 `mit`。

3. 闭合夹爪并回到安全零位：

```bash
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
```

4. 失能并退出：

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

---

## 示例脚本

所有示例都假设已经启动 `reBotArmController`：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

示例已注册为 ROS2 可执行入口，可以直接通过 `ros2 run` 调用。

源文件位于：

```text
src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py
src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py
src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py
src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py
```

### move_to.py

关节空间绝对角移动示例。一次性控制 6 个电机，参数为 6 个绝对关节角，单位 rad：

```bash
ros2 run rebotarmcontroller MoveTo -- \
  0.20 -0.20 -0.20 -0.20 0.10 -0.10 \
  --duration 8.0
```

一次性控制 1 个电机，参数为目标关节名和绝对关节角，单位 rad：

```bash
ros2 run rebotarmcontroller MoveTo -- --joint joint3 --position -0.20 --duration 5.0
```

### move_to_pose.py

末端位姿移动示例。

```bash
ros2 run rebotarmcontroller MoveToPose -- --x 0.30 --y 0.0 --z 0.30 --qw 1.0 --duration 2.0
```

### gravity_compensation.py

重力补偿示例。

```bash
ros2 run rebotarmcontroller GravityCompensation
```

脚本启动时会先调用 `/rebotarm/enable`，再启动重力补偿。按 `Ctrl+C` 退出时，
脚本会依次调用 `/rebotarm/gravity_compensation/stop`、`/rebotarm/safe_home`
和 `/rebotarm/disable`，让机械臂回到安全零位后失能。

对应底层服务：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

### gripper_control.py

交互式夹爪开闭示例。

```bash
ros2 run rebotarmcontroller GripperControl
```

运行后输入：

```text
o / open    打开夹爪
c / close   闭合夹爪
q / quit    退出
```
---

## API 文档

完整 ROS2 API 已整理到独立文档：[API_zh.md](API_zh.md)。

其中包含：

- topic、service、action 的完整列表和类型
- `/rebotarm` 命名空间、QoS、单位和状态机约定
- `JointMitCmd`、`JointPosVelCmd`、`ArmStatus`、`MoveToPose` 等自定义接口说明
- 末端位姿移动、夹爪控制、重力补偿、低层 command 的调用示例
- 上层集成和多机械臂命名注意事项

---

## 配置说明

`rebotarm_bringup/config/` 提供默认配置：

| 文件 | 说明 |
|---|---|
| `rebotarm_hardware.yaml` | ROS2 上层硬件选择和覆盖配置，可选择 DM / RS 并覆盖 SDK 参数 |
| `driver_params.yaml` | ROS 参数示例 |

常用 launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `hardware_config` | bringup 内置 `rebotarm_hardware.yaml` | ROS2 上层硬件配置路径 |
| `model` | 空字符串 | 留空使用 `default_model`，可设为 `dm` 或 `rs` |
| `channel` | 空字符串 | 留空使用 YAML，非空时覆盖通信通道 |
| `joint_state_rate` | `100.0` | `/rebotarm/joint_states` 发布频率 |
| `cmd_arbitration` | `reject` | 轨迹运行中 arm joint 低层 cmd 仲裁，`reject` 或 `preempt`；gripper 低层 cmd 不抢占 arm 轨迹 |
| `arm_namespace` | `rebotarm` | ROS 命名空间前缀 |
| `frame_id` | `base_link` | 机械臂基座坐标系，预留给 TF、视觉和规划集成 |
| `ee_frame_id` | `end_link` | 末端坐标系，预留给 TF、视觉和规划集成 |
| `use_rviz` | `false` | 是否启动 bringup RViz |
| `disable_after_safe_home` | `true` | 该参数控制 safe home 完成后是否失能电机 |

`rebotarm_hardware.yaml` 默认型号配置：

| 型号 | 默认通道 | arm 控制模式 | 夹爪限位 |
|---|---|---|---|
| `dm` | `/dev/ttyACM0` | `posvel` | open `-5.0`，close `0.0` |
| `rs` | `can0` | `mit` | open `5.0`，close `0.0` |

---

## MoveIt 2

MoveIt 2 是用于机械臂运动规划的框架，这里主要负责逆解、碰撞检测、轨迹规划和轨迹执行，
并通过独立的 demo 包将应用流程与底层驱动隔离开。
更多内容可参考官方 [MoveIt 2 文档](https://moveit.picknik.ai/main/index.html)。

MoveIt 相关内容集中在两个包：

| 包 | 作用 |
|---|---|
| `rebotarm_moveit_config` | 机械臂模型、SRDF、运动学、joint limits、controller 和 RViz 配置 |
| `rebotarm_moveit_demos` | 基于 MoveIt 2 的应用 demo |

MoveIt 环境使用 `ros2_control` 的模拟硬件和 `move_group` 进行规划执行，适合在 RViz
中验证模型、IK、轨迹规划和 demo 流程。

本仓库同样提供了硬件接口的支持。接入真实硬件前，请先确认机械臂零点配置、关节方向、限位、
速度和夹爪开闭范围相关配置准确或者保持仓库默认配置。

### MoveIt 环境配置

先确认已经加载 ROS2 环境。下面的命令会使用当前 `ROS_DISTRO` 安装对应版本依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-moveit-configs-utils \
  ros-${ROS_DISTRO}-moveit-kinematics \
  ros-${ROS_DISTRO}-moveit-planners-ompl \
  ros-${ROS_DISTRO}-moveit-simple-controller-manager \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-xacro
```

MoveIt 相关包和 demo 已包含在本工作空间中，安装依赖后重新构建：

```bash
cd your/path/to/rebotarm_ros2
colcon build --symlink-install
source install/setup.bash
```

验证 MoveIt 包和 demo 入口：

```bash
ros2 pkg list | grep rebotarm_moveit
ros2 pkg executables rebotarm_moveit_demos
```

期望至少能看到如下两个可执行 Demo：

```text
rebotarm_moveit_demos draw_square
rebotarm_moveit_demos pick_place
```

### 使用 MoveIt

MoveIt 的规划功能需要基于 RViz GUI 或者通过节点调用，可以适用于仿真或真实场景。

#### 在仿真环境使用 MoveIt

MoveIt 通过 ros2_control 虚拟硬件接口实现 RViz 中的仿真，首先启用

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_config demo.launch.py
```
注意 RS 版本机械臂在没有修改默认类型机械臂的情况下需要指定：

```bash
ros2 launch rebotarm_moveit_config demo.launch.py model:=rs
```

默认会启动：

- `move_group`
- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `rebotarm_controller`
- `gripper_controller`
- RViz MoveIt MotionPlanning 插件

RViz 界面会自动弹出并加载机械臂的urdf模型，可以通过左侧的 GUI 控制面板对机械臂的运动进行控制。

#### 使用 MoveIt 控制 reBotArm

在实际场景中使用 MoveIt 控制 reBotArm 需要先启动带有硬件接口的控制器而不再是虚拟控制器，
再启动针对实际场景的 MoveIt 环境：

```bash
ros2 launch rebotarm_bringup driver.launch.py
```

另开终端：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_config hardware.launch.py
```

再次重申，在真实硬件上运行任何 demo 前，请确保机械臂工作空间内无人和障碍物，先在 RViz 中确认规划路径，并随时准备停止控制器。

### 运行画矩形 demo

先启动 MoveIt 环境，再另开一个终端运行：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_demos draw_square.launch.py
```

`draw_square` 会控制 `gripper_tcp` 遍历同一平面矩形的四个角点。默认参数在：

```text
src/rebotarm_moveit_demos/config/draw_square.yaml
src/rebotarm_moveit_demos/config/draw_square_rs.yaml
```

常用参数：

| 参数 | 说明 |
|---|---|
| `start_point` | demo 开始前复位到的关节位置 |
| `rectangle_center` | 矩形中心点，坐标系为 `base_link` |
| `rectangle_width` / `rectangle_height` | 矩形宽高，单位 m |
| `tcp_rpy` | 末端姿态，默认让夹爪竖直朝下 |
| `tcp_yaw_offsets` | IK 备选 yaw，用于避免 joint6 大幅绕转 |

### 运行抓取放置 demo

先启动 MoveIt 环境，再另开一个终端运行：

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_demos pick_place.launch.py
```

默认参数在：

```text
src/rebotarm_moveit_demos/config/pick_place.yaml
src/rebotarm_moveit_demos/config/pick_place_rs.yaml
```

常用参数：

| 参数 | 说明 |
|---|---|
| `ready_point` | 抓取前后使用的预备关节位置 |
| `pick_position` | 物体底面中心位置，坐标系为 `base_link` |
| `pick_tcp_rpy` / `place_tcp_rpy` | 抓取和放置时的末端姿态 |
| `object_dimensions` | MoveIt 场景中物体尺寸，单位 m |
| `max_gripper_width` | 夹爪最大总开口，默认 `0.09m` |
| `open_gripper_position` / `grasp_gripper_position` / `closed_gripper_position` | 仿真夹爪单侧关节位置 |
| `hardware_open_gripper_position` / `hardware_closed_gripper_position` | 硬件夹爪电机开闭位置 |

### MoveIt 配置文件

| 文件 | 说明 |
|---|---|
| `rebotarm_moveit_config/config/rebotarm.urdf.xacro` | MoveIt 使用的机器人模型 |
| `rebotarm_moveit_config/config/rebotarm.srdf` | MoveIt group、end effector、默认状态等语义配置 |
| `rebotarm_moveit_config/config/rebotarm_rs.urdf.xacro` | MoveIt 使用的 RS 机器人模型 |
| `rebotarm_moveit_config/config/rebotarm_rs.srdf` | RS MoveIt group 和碰撞语义配置 |
| `rebotarm_moveit_config/config/kinematics.yaml` | IK solver 配置 |
| `rebotarm_moveit_config/config/joint_limits.yaml` | MoveIt 规划使用的关节限位 |
| `rebotarm_moveit_config/config/moveit_controllers.yaml` | DM/RS 共用的 MoveIt trajectory execution controller 配置 |
| `rebotarm_moveit_config/config/ros2_controllers.yaml` | DM/RS 共用的 ros2_control controller 配置 |
| `rebotarm_moveit_config/config/initial_positions.yaml` | ros2_control 模拟硬件初始关节位置 |
| `rebotarm_moveit_demos/config/draw_square.yaml` | 画矩形 demo 参数 |
| `rebotarm_moveit_demos/config/draw_square_rs.yaml` | RS 画矩形 demo 参数 |
| `rebotarm_moveit_demos/config/pick_place.yaml` | 抓取放置 demo 参数 |
| `rebotarm_moveit_demos/config/pick_place_rs.yaml` | RS 抓取放置 demo 参数 |

---

## 演示项目

以下演示在仿真驱动（`fake_dm_bringup.launch.py`）和真机上均可运行，接口完全一致。
运行前请确保驱动已启动：

```bash
source /opt/ros/jazzy/setup.bash   # Ubuntu 22.04 请换成 humble
source install/setup.bash
ros2 launch rebotarm_bringup fake_dm_bringup.launch.py   # 虚拟驱动 + RViz
```

### 打招呼（rebotarm_greeting）

三位置点头问候，动作剧本在 `src/rebotarm_greeting/config/greeting.yaml`。

```bash
ros2 launch rebotarm_greeting greeting.launch.py
# 触发：
ros2 topic pub --once /rebotarm/greeting/start std_msgs/msg/Bool "{data: true}"
```

### 跳舞（rebotarm_dance）

约 38s 的表演：开场抬臂、机械街舞两圈（TCP 钉在空中、身体律动）、沿矩形的
空间滑步、指天定格收尾。所有关节轨迹均离线预计算（30Hz），运行时不在线解 IK。

```bash
ros2 launch rebotarm_dance dance.launch.py                    # 等待触发
ros2 launch rebotarm_dance dance.launch.py auto_start:=true   # 自动开演
# 触发：
ros2 topic pub --once /rebotarm/dance/start std_msgs/msg/Bool "{data: true}"
```

调整动作顺序、时长或触发方式：改 `src/rebotarm_dance/config/dance.yaml`；
改舞步本身需要重跑 `precompute_dance.py` 并重新编译，详见
`src/rebotarm_dance/README.md`。

### 手势识别（rebotarm_vision）

摄像头 → MediaPipe Hands（21 关键点）→ 纯几何规则分类（不训练模型）→ 发布
`/rebotarm/gesture_cmd`（`rock` / `paper` / `heart` / `none`）。额外依赖：

```bash
pip install --user --break-system-packages "mediapipe==0.10.14" "numpy==1.26.4"
```

`numpy` 必须保持 <2，否则会破坏 ROS2/pinocchio。

```bash
# 独立测试脚本（不进 ROS，用于调阈值），按 r 打印比值，q 退出：
python3 src/rebotarm_vision/scripts/test_gesture.py

# ROS 节点：
ros2 launch rebotarm_vision vision.launch.py                    # 带调试窗口
ros2 launch rebotarm_vision vision.launch.py show_debug:=false  # 无显示环境
```

### 手势动作（rebotarm_gesture_actions）

识别到手势后回放对应的预计算 TCP 轨迹：

| 手势 | 轨迹 | 动作 |
|---|---|---|
| ✊ 石头 | `rock_push` | TCP 沿基座 X 轴前后 ±6cm，快出慢收 ×3 |
| ✋ 布 | `paper_figure8` | Y-Z 竖直面画 26×14cm 大 8 字 ×2，手腕同步扭转 |
| 🫶 比心 | `heart_draw` | 两笔画心，高 22cm |

完整链路（视觉 → 分发 → 回放），需先启动驱动：

```bash
ros2 launch rebotarm_vision vision.launch.py                    # 终端2
ros2 launch rebotarm_gesture_actions gesture_actions.launch.py  # 终端3
```

分发节点为边沿触发：同一手势持续举着只执行一次，放下手（识别为 `none`）后
再次举起才触发下一次。没有摄像头时也可以手动触发：

```bash
ros2 topic pub --once /rebotarm/action/play std_msgs/msg/String "{data: rock_push}"
```

重新生成轨迹的方法见 `src/rebotarm_gesture_actions/README.md`。

---

## FAQ / 排障

### `ros2: command not found`

当前终端没有 ROS2 环境，或者尚未安装 ROS2。先安装 ROS2，然后在每个新终端中加载对应发行版环境：

```bash
source /opt/ros/humble/setup.bash
```

如果使用 Jazzy，将 `humble` 替换为 `jazzy`。

如果希望新终端自动加载 ROS2 环境，可以把 source 命令写入 `~/.bashrc`：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 找不到 ROS2 可执行文件

如果 `ros2 run` 或 `ros2 launch` 找不到 package 或 executable，通常是当前终端没有加载本工作空间。
需要先构建，再 source 工作空间：

```bash
cd your/path/to/rebotarm_ros2
colcon build --symlink-install
source install/setup.bash
```

可以用下面的命令检查入口是否已经注册：

```bash
ros2 pkg executables rebotarmcontroller
ros2 pkg executables rebotarm_moveit_demos
```

### 找不到串口

如果启动时报：

```text
open serial port /dev/ttyACM0 failed: No such file or directory
```

说明默认串口不存在。先查看实际设备：

```bash
ls /dev/ttyACM*
```

然后用 `channel:=...` 覆盖：

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

RS 需要确认 SocketCAN 接口已经启动：

```bash
ip -details link show can0
sudo ip link set can0 up type can bitrate 1000000
ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can0
```

### 权限不足

如果串口存在但无权限：

```bash
sudo usermod -a -G dialout $USER
```

重新登录后生效。

### RViz 模型不显示

确认 URDF mesh 路径已经是：

```text
package://rebotarm_bringup/description/...
```

### FastDDS SHM 端口提示

如果终端出现类似：

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7002: open_and_lock_file failed
```

通常是之前的 ROS2 进程异常退出后，FastDDS shared memory 锁文件残留。服务和 action
能正常响应时，这个提示一般不影响控制。需要清理时，先停掉相关 ROS2 进程，再执行：

```bash
pkill -f ros2
pkill -f reBotArmController
rm -f /dev/shm/fastrtps_port*
```

如果希望临时绕开 shared memory transport，可在启动 ROS2 前设置：

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

---

## 各功能启动操作速查

本节集中列出日常操作。命令以当前工作空间 `/home/syh/rebotarm_ros2`、ROS 2 Jazzy 为例；
其他安装位置请替换目录，Humble 用户将 `jazzy` 改为 `humble`。

### 1. 每个终端先加载环境

```bash
cd /home/syh/rebotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

若终端自动进入 Conda，先 `conda deactivate`，直到 `which python3` 指向系统 Python。
本机 Jazzy 使用 `/usr/bin/python3`（Python 3.12）。同一组节点使用相同的 `ROS_DOMAIN_ID`。
首次构建或新增包后执行，再在其他终端重新 source：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

| 想做什么 | 需要启动的功能 | 适用范围 |
|---|---|---|
| 无真机观察机械臂 | 下方 DM / RS 仿真驱动之一 | DM / RS |
| 招手、跳舞 | 驱动 + 对应演示节点 + 触发命令 | 本仓库演示默认 DM |
| 手势触发动作 | 驱动 + 摄像头识别 + 手势动作调度 | DM |
| 手掌位置跟随 | 驱动 + palm_follow | DM |
| 本地语音控制及字幕 | 驱动 + voice_control + listen 服务 | DM |
| 轨迹规划、画矩形、抓取放置 | MoveIt demo，或驱动 + MoveIt hardware | DM / RS，model 保持一致 |
| RS 物理仿真、颜色识别 | MuJoCo 模型资源 + mujoco_rs | RS，需额外依赖 |
| MCP 工具 / 文本代理 | 可用 ROS 接口 + MCP；文本代理另需模型服务 | 默认命名空间 rebotarm_rs |

每个长期运行的 launch 占用一个终端；触发命令在另一个终端执行。
**同一命名空间只启动一个驱动；舞蹈、招手、手势轨迹、手掌跟随、语音动作、MoveIt 等
运动控制功能切换运行。** 现有低层关节话题没有跨模块的全局互斥锁。

### 2. 启动仿真或真机驱动（选择一种）

DM 仿真（默认带 RViz，也是下面 DM 演示的推荐入口）：

```bash
ros2 launch rebotarm_bringup fake_dm_bringup.launch.py
```

RS 简易仿真：

```bash
ros2 launch rebotarm_bringup fake_rs_bringup.launch.py
```

DM 真机（串口按实际设备修改）：

```bash
ros2 launch rebotarm_bringup bringup.launch.py model:=dm channel:=/dev/ttyACM0 use_rviz:=true
```

RS 真机（先按硬件说明配置 SocketCAN）：

```bash
ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can0 arm_namespace:=rebotarm_rs use_rviz:=true
```

只运行真机驱动、不启动模型与 RViz：

```bash
ros2 launch rebotarm_bringup driver.launch.py model:=dm channel:=/dev/ttyACM0
```

基础状态和服务（RS 命令改为 `/rebotarm_rs/...`）：

```bash
ros2 topic echo /rebotarm/arm_status
ros2 service call /rebotarm/enable std_srvs/srv/Trigger '{}'
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger '{}'
```

`safe_home` 会运动到安全位，真机默认随后失能。仅在现场准备就绪时调用运动服务。
底层单关节、Pose、夹爪和重力补偿示例见本文“示例脚本 / Example Scripts”及 [API 文档](API_zh.md)。

### 3. 打招呼与跳舞

先保持 DM 驱动运行，以下两种演示分别启动。

```bash
# 终端二：打招呼，等待触发
ros2 launch rebotarm_greeting greeting.launch.py
```

```bash
# 终端三：触发一次打招呼
ros2 topic pub --once /rebotarm/greeting/start std_msgs/msg/Bool '{data: true}'
```

停止打招呼节点后，可切换跳舞：

```bash
# 终端二：跳舞，等待触发
ros2 launch rebotarm_dance dance.launch.py
```

```bash
# 终端三：触发一次跳舞
ros2 topic pub --once /rebotarm/dance/start std_msgs/msg/Bool '{data: true}'
```

两种 launch 都支持 `auto_start:=true`，启动后立即演示一次。
舞蹈还支持 `track_speed:=1.0`，默认倍率为 `2.0`。

### 4. 摄像头手势识别与手势动作

首次使用安装视觉依赖（使用系统 Python，保持本项目 NumPy 版本）：

```bash
/usr/bin/python3 -m pip install --user --break-system-packages 'mediapipe==0.10.14' 'numpy==1.26.4'
```

先保持 DM 驱动运行：

```bash
# 终端二：摄像头识别
ros2 launch rebotarm_vision vision.launch.py camera_index:=0 show_debug:=true
```

```bash
# 终端三：手势调度 + 轨迹播放器
ros2 launch rebotarm_gesture_actions gesture_actions.launch.py
```

握拳 → 出拳；张开手掌 → 画八字；双手比心 → 画爱心。
持续保持同一手势只执行一次，放下手并等待冷却后再次举起才能重新触发。
仅查看识别结果、不执行动作时，不启动终端三：

```bash
ros2 topic echo /rebotarm/gesture_cmd
```

没有摄像头时，保持播放器运行也可以手动触发：

```bash
ros2 topic pub --once /rebotarm/action/play std_msgs/msg/String '{data: heart_draw}'
```

轨迹名还有 `rock_push`、`paper_figure8`、`paper_circle`。
摄像头编号可修改 `camera_index`；不需要调试窗口时使用 `show_debug:=false`。

### 5. 手掌位置跟随

停止语音动作、手势播放器、舞蹈等控制节点，保持 DM 驱动运行，然后执行：

```bash
ros2 launch rebotarm_vision palm_follow.launch.py camera_index:=0 show_debug:=true
```

单手张开稳定后获取基准，移动手掌即可控制末端左右、上下移动；握拳、双手入镜或
手掌消失会暂停。重新张开时从当前位置重新获取基准，不自动回零。
建议先把机械臂置于舒展的悬空姿态，避免全零奇异位形。该功能不做环境/自碰撞规划。
详细参数见 [视觉与手掌跟随说明](src/rebotarm_vision/README.md)。

### 6. 本地免费 Whisper 语音控制

无需 OpenAI API Key，无 API 调用费用，音频不上传。首次安装依赖和下载模型：

```bash
sudo apt-get install -y libportaudio2 fonts-noto-cjk python3-pil
/usr/bin/python3 -m pip install --user --break-system-packages 'faster-whisper>=1.1,<2' 'sounddevice>=0.4' 'numpy==1.26.4' Pillow
ros2 run rebotarm_voice download_model --model base
```

本机已安装依赖并下载 `base` 模型，无需重复下载。模型下载完毕后可断网识别。
先保持 DM 仿真或真机驱动运行，停止其他运动控制节点。

```bash
# 终端二：先只识别、不发送动作
ros2 launch rebotarm_voice voice_control.launch.py
```

等待终端显示 `READY: local Whisper loaded` 后，在另一个终端触发：

```bash
# 终端三：每次调用录音 4 秒；命令执行后立即说话
ros2 service call /rebotarm/voice/listen std_srvs/srv/Trigger '{}'
```

| 口令 | 动作 |
|---|---|
| 出拳 | 出拳 |
| 画八字 | 画八字 |
| 画爱心 | 画爱心 |
| 画圆 | 画圆 |

直接说动作命令即可，无需“机械臂”前缀；可加“请”，例如“请画爱心”。
服务返回 `Recording started` 只表示开始录音；录音结束并完成推理后才输出识别文字。
确认识别效果后，停止终端二中的语音 launch，替换为执行模式：

```bash
ros2 launch rebotarm_voice voice_control.launch.py execute_motion:=true
```

如果已有 `gesture_actions.launch.py` 中的播放器，启动语音时加 `start_player:=false`，
不要重复启动播放器；专门做语音演示时建议停掉手势链路，让语音 launch 管理播放器。

查看识别结果、状态及麦克风列表：

```bash
ros2 topic echo /rebotarm/voice/transcript
ros2 topic echo /rebotarm/voice/status
/usr/bin/python3 -m sounddevice
```

`DRY_RUN`：仅预览；`REQUESTED`：已发送动作请求；`IGNORED`：口令无效、过期或当前不可执行；
`ERROR`：查看语音节点日志。动作执行中与冷却期间不接受新动作。
本功能目前只适配 DM，不能仅通过更改命名空间用于 RS。

### 7. 把语音字幕叠加到 RViz 画面底部

语音 launch 默认启动字幕节点。字幕为红色、透明背景，直接叠加在机械臂三维画面底部居中。
需要 `rviz_2d_overlay_msgs` 与 `rviz_2d_overlay_plugins`（本机已构建）；其他机器的
安装或固定版本源码构建步骤见 [语音模块说明](src/rebotarm_voice/README.md#rviz-底部红色叠加字幕)。

重新打开使用默认配置的 DM 仿真 RViz，即可加载 **Voice Subtitles**。
自定义配置请 Add → `rviz_2d_overlay_plugins/TextOverlay`，Topic 设置为：

```text
/rebotarm/voice/subtitles
```

三个 `Overtake ... Properties` 保持 false。旧版左侧 Image 字幕面板应删除；
当前话题类型为 `rviz_2d_overlay_msgs/msg/OverlayText`。
如果语音 launch 没有启动字幕节点，可单独执行：

```bash
ros2 run rebotarm_voice rviz_subtitles
```

字幕节点只保留一份，支持 `show_subtitles:=false` 关闭自动启动。
每段录音转写完成后更新文字，当前不是说话时逐字显示。

### 8. MoveIt 规划与演示

独立 MoveIt 仿真（自行启动模拟控制器；不要再启动前面的 fake 驱动）：

```bash
ros2 launch rebotarm_moveit_config demo.launch.py model:=dm
```

RS 改为 `model:=rs`。若接入真机，则用以下两个终端替代独立仿真：

```bash
# 终端一：真实驱动
ros2 launch rebotarm_bringup driver.launch.py model:=dm channel:=/dev/ttyACM0
```

```bash
# 终端二：连接现有驱动的 MoveIt
ros2 launch rebotarm_moveit_config hardware.launch.py model:=dm arm_namespace:=rebotarm
```

规划环境准备好后，在新终端选择一个演示：

```bash
ros2 launch rebotarm_moveit_demos draw_square.launch.py model:=dm
# 或：
ros2 launch rebotarm_moveit_demos pick_place.launch.py model:=dm
```

RS 的驱动、MoveIt 和演示统一使用 `model:=rs`，驱动与 MoveIt 的 `arm_namespace` 保持一致。
MoveIt 的安装与交互操作见本文原有 MoveIt 2 章节。

### 9. RS MuJoCo 物理仿真（额外环境）

此入口需要额外安装 MuJoCo、准备上游 RS 模型和 meshes，不等同于前面的 fake 仿真。
仓库未附带上游资源，也没有部分旧文档所说的 `scripts/start_rs_sim.sh` 等脚本。
请先将 `LAN-GER/reBot-B601-RS-for-mujoco_sim` 的配套资源准备到
`third_party/reBot-B601-RS-for-mujoco_sim/`，检查场景 XML 的 include/mesh 相对路径。
本工作空间采用 `--symlink-install`，也可显式指定能正确解析资源的 `model_path`。

```bash
ros2 launch rebotarm_mujoco_rs mujoco_rs.launch.py arm_namespace:=rebotarm_rs simulation_mode:=physics use_viewer:=true
```

默认启动任务接口、场景相机和颜色检测。只同步已有 ROS 关节状态、不开启任务工具：

```bash
ros2 launch rebotarm_mujoco_rs mujoco_rs.launch.py arm_namespace:=rebotarm_rs simulation_mode:=kinematic enable_task_tools:=false use_viewer:=true
```

后一种模式需要已有 `/rebotarm_rs/joint_states` 发布者。任务工具与其他驱动可能提供同名
service/action，应按使用模式选择，避免重复启动。查看颜色检测：

```bash
ros2 topic echo /rebotarm_rs/vision/color_blocks/detections
```

接口说明见 [MuJoCo 模块](src/rebotarm_mujoco_rs/README.md)。

### 10. MCP 与文本 Agent（可选）

需要额外的 `fastmcp` 依赖，以及能调用 ROS 的 Python 解释器。以下使用系统解释器，
显式覆盖 launch 内遗留的其他工作空间虚拟环境路径：

```bash
/usr/bin/python3 -m pip install --user --break-system-packages fastmcp
ros2 launch rebotarm_agent rebotarm_mcp.launch.py python_executable:=/usr/bin/python3 arm_namespace:=rebotarm_rs motion_mode:=locked
```

MCP 地址为 `http://127.0.0.1:8081/mcp`。需先准备对应命名空间的驱动或仿真接口。
`locked` 默认禁止运动工具；在准备好的安全仿真中可替换为 `motion_mode:=allow`。

文本代理与本地 Whisper 独立，需要另行配置兼容的聊天模型服务。
下面环境变量填入你实际使用的服务地址、模型名和密钥；云端聊天服务可能收费，
这些配置不是本地语音功能的依赖：

```bash
export REBOTARM_LLM_BASE_URL='你的聊天模型服务地址'
export REBOTARM_LLM_MODEL='你的模型名'
export REBOTARM_LLM_API_KEY='你的聊天服务密钥'
ros2 launch rebotarm_agent rebotarm_text_agent.launch.py python_executable:=/usr/bin/python3
```

如需网页 Dashboard，用以下命令替代上面的文本交互入口：

```bash
ros2 run rebotarm_agent rebotarm_text_agent --http-server --http-host 127.0.0.1 --http-port 8082
```

浏览器打开 `http://127.0.0.1:8082`。更多工具说明见 [Agent 模块](src/rebotarm_agent/README.md)。

### 11. 结束与切换功能

在对应长期运行节点的终端按 `Ctrl+C`。切换运动功能前，先停掉旧运动控制节点；
驱动和 RViz 可按需要保留。修改代码或启动配置后重新构建相应包，并在每个终端
重新 `source install/setup.bash`、重启对应节点。`Ctrl+C` 不是硬件急停。


### 一键停止与缓慢复位（真机控制器）

构建并加载环境后打开按钮窗口（需要 `python3-tk`）：

```bash
colcon build --packages-select rebotarmcontroller
source install/setup.bash
ros2 run rebotarmcontroller SafetyPanel
```

点击“停止并缓慢复位”会打断动作、保持当前位置，然后以五次平滑曲线回到各关节零位，目标速度不超过 0.1 rad/s。夹爪保持触发时的位置。点击“仅停止”或在窗口获得焦点时按空格，可中断回零并保持当前位置。

也可直接调用：

```bash
ros2 service call /rebotarm/stop_and_home std_srvs/srv/Trigger '{}'
ros2 service call /rebotarm/safety_stop std_srvs/srv/Trigger '{}'
```

触发后普通运动指令被锁止，复位完成仍保持锁止和电机位置保持。再次点击复位只停止，不会重新启动回零。现场检查后重启控制器才能恢复控制。现有 `safe_home` 服务也使用此限速流程。关闭已锁止的控制器时直接失能，不再自动运动。

这是软件停止，不能替代独立硬件急停；通信失效时软件无法保证停车。自动回零没有避障，请预先保证回零路径和零位周围无人、无障碍物。0.1 rad/s 为目标轨迹限速，不是经认证的实际速度限制。服务成功表示已开始处理，需通过状态主题确认复位过程；尚需在隔离区域进行真机验收。


### RViz 内嵌停止面板

默认 RViz 布局现在包含 **Arm Safety** 停靠面板，无需单独打开 Tk 窗口。构建并重新加载环境后启动 DM 仿真：

```bash
colcon build --packages-select rebotarm_rviz_panel rebotarmcontroller rebotarm_bringup rebotarm_moveit_config
source install/setup.bash
ros2 launch rebotarm_bringup fake_dm_bringup.launch.py
```

RS 仿真使用 `fake_rs_bringup.launch.py use_rviz:=true`。面板随启动文件自动选择机械臂命名空间，也可在面板中修改后按 Enter。点击红色“仅停止 / 中断复位”保持当前位置；点击“停止并缓慢复位”以不超过 0.1 rad/s 回零。状态栏显示 `SAFETY_HOMING`（复位中）或 `SAFETY_STOPPED`（停止并锁止）。再次点击复位只会停止。夹爪保持原位置，普通指令被锁止，检查后重启驱动恢复控制。

旧 RViz 布局可通过 **Panels → Add New Panel → rebotarm_rviz_panel/SafetyPanel** 添加面板。停止服务未启动时按钮禁用；底层仅有 MoveIt `ros2_control` 的 demo 不提供这两个服务，需要使用上述 fake bringup 或真机控制器。服务响应只表示请求被受理，状态栏用于查看后续状态。软件按钮不替代硬件急停，自动回零路径不带避障。


仿真生命周期：关闭上述 DM/RS 仿真的 RViz 窗口会同时退出该次 launch 的驱动与状态发布节点；驱动异常退出也会结束整组进程。相同 ROS 域、相同机械臂命名空间在本机只允许一个新版仿真驱动，重复启动会报错退出，避免不同状态交替显示以及同名停止服务由不同驱动接收。升级前遗留的旧进程不具备互斥保护，需先在旧仿真终端 Ctrl+C 完整退出再启动。跨机器、旧版本驱动或其他状态发布者仍需单独检查。

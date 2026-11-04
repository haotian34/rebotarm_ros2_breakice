# reBot Arm ROS2 SDK

<p align="center">
  <img src="./media/rebot_arm_b601.png" alt="reBot Arm" width="720">
</p>

<p align="center">
  <strong>ROS2 · Arm Control · Gripper Control · Trajectory Interfaces · RViz · Open Source</strong>
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
    <a href="./API_zh.md">API Reference</a>
  </strong>
</p>

**启动操作入口：** [各功能启动操作速查](#各功能启动操作速查)（仿真、真机、手势、手掌跟随、本地语音、RViz 字幕、MoveIt、MuJoCo、Agent）。

---

## About This Repository (Modified Fork)

This repository is a personal modified fork of Seeed's official open-source project
[reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2).
On top of the original version, it adds/modifies:

| Addition | Description |
|---|---|
| `rebotarm_greeting` | Greeting demo: three-position nod, topic-triggered (see [Demo Projects](#demo-projects)) |
| `rebotarm_dance` | Dance demo: ~38 s routine with offline precomputed trajectories (see [Demo Projects](#demo-projects)) |
| `rebotarm_vision` | Gesture recognition: camera + MediaPipe, publishes `/rebotarm/gesture_cmd` |
| `rebotarm_gesture_actions` | Gesture-triggered TCP trajectory playback (rock / paper / heart) |
| `fake_dm_bringup.launch.py` | DM fake-hardware bringup (runs without a real arm) |

### Installation

1. **Environment setup**: follow steps 1, 2 and 4 of the official wiki
   (install ROS2, dependencies and motorbridge). The two `git clone` commands
   from official steps 3 and 5 are already merged into step 2 below — no need
   to run them separately:
   <https://wiki.seeedstudio.com/cn/rebot_arm_b601_dm_ros2_integration/>
2. **Clone this repository and the low-level SDK** (covers official steps 3
   and 5. Note: use THIS repo's URL for the workspace, but keep Seeed's official
   URL for the `reBotArm_control_py` SDK — third_party is not included here):

   ```bash
   git clone https://github.com/haotian34/rebotarm_ros2_breakice.git
   cd rebotarm_ros2_breakice
   mkdir -p third_party
   git clone https://github.com/Seeed-Projects/reBotArm_control_py.git third_party/reBotArm_control_py
   ```

3. **Build and run**:

   ```bash
   source /opt/ros/jazzy/setup.bash   # use humble on Ubuntu 22.04
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch rebotarm_bringup fake_dm_bringup.launch.py
   ```

4. **Gesture recognition dependency** (optional, only for the vision demo):

   ```bash
   pip install --user --break-system-packages "mediapipe==0.10.14" "numpy==1.26.4"
   ```

   `numpy` must stay <2, otherwise it breaks ROS2/pinocchio.

Everything else works the same as the official version, see below.

---

## Overview

Current version: `v0.3.0`

`rebotarm_ros2` is the ROS2 SDK workspace for reBot Arm B601 DM and RS models. It
wraps the current `reBotArm_control_py` Python control library into ROS2 topics,
services and actions, providing a unified entry point for development, planning,
RViz visualization, gravity compensation and per-motor debugging.

The workspace contains five core ROS2 packages:

| Package | Purpose |
|---|---|
| `rebotarm_msgs` | Custom msg / srv / action interfaces |
| `rebotarmcontroller` | Driver node package, including `reBotArmController` |
| `rebotarm_bringup` | Launch files, configs, URDF and RViz resources |
| `rebotarm_moveit_config` | MoveIt 2 config, SRDF, ros2_control and RViz config |
| `rebotarm_moveit_demos` | MoveIt 2 demos, including pick-place and rectangle drawing |

plus four application demo packages (see [Demo Projects](#demo-projects)):

| Package | Purpose |
|---|---|
| `rebotarm_greeting` | Topic-triggered three-position nod greeting |
| `rebotarm_dance` | Dance routine with offline precomputed joint trajectories |
| `rebotarm_vision` | MediaPipe-based gesture recognition (rock / paper / heart) |
| `rebotarm_gesture_actions` | Gesture-to-trajectory dispatcher and player |

---

## Features

- Publishes arm status: `/rebotarm/joint_states`, `/rebotarm/arm_status`
- Provides core services: `enable`, `disable`, `set_zero`, `safe_home`
- Supports Cartesian targets: `MoveToPoseIK` service, `MoveToPose` action
- Supports the standard `control_msgs/action/FollowJointTrajectory` interface
- Supports gripper control: `SetGripper` service, `GripperCommand` action
- Supports single-joint commands: `JointMitCmd`, `JointPosVelCmd`

---

## Requirements

| Item | Requirement |
|---|---|
| Arm | reBot Arm B601 DM or RS |
| Communication | DM: USB serial bridge; RS: SocketCAN interface |
| Host | Ubuntu 22.04+, ROS2, Python 3.10+ |
| Camera | USB camera (only for the gesture-recognition demo) |

Wiring:

1. Connect the USB2CAN serial bridge to the arm CAN bus.
2. Connect the gripper motor to the same CAN bus.
3. Plug the USB2CAN bridge into the host and check the device name:

```bash
ls /dev/ttyACM*
```

For temporary serial access:

```bash
sudo chmod 666 /dev/ttyACM0
```

RS uses SocketCAN by default. Bring up the CAN interface before starting the ROS2 driver:

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

---

## Development Setup

### Step 1. Install ROS2

Choose and install the appropriate ROS2 distribution from the
[official ROS getting started guide](https://www.ros.org/blog/getting-started/).

### Step 2. Clone the ROS2 workspace

Preferred upstream repository:

```bash
git clone https://github.com/Seeed-Projects/reBotArmController_ROS2.git rebotarm_ros2
cd rebotarm_ros2
```

Current development repository:

```bash
git clone https://github.com/EclipseaHime017/reBotArmController_ROS2.git rebotarm_ros2
cd rebotarm_ros2
```

### Step 3. Install motorbridge

Install `motorbridge` from the official PyPI index:

```bash
python3 -m pip install --user --index-url https://pypi.org/simple motorbridge
```

### Step 4. Clone the low-level SDK

```bash
mkdir -p third_party
git clone https://github.com/vectorBH6/reBotArm_control_py.git third_party/reBotArm_control_py
```

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

Verify installed executables:

```bash
ros2 pkg executables rebotarmcontroller
```

Expected output:

```text
rebotarmcontroller reBotArmController
rebotarmcontroller GravityCompensation
rebotarmcontroller GripperControl
rebotarmcontroller MoveTo
rebotarmcontroller MoveToPose
```

---

## Directory Layout

```text
rebotarm_ros2/
├── README.md
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

## Quick Start

Before using the robot, note the following: **The arm controller has a high
degree of freedom. Before enabling the controller or powering the arm, make
sure the workspace is clear of people and obstacles. Review every motion
command carefully to avoid accidents. Dangerous operation is strictly
prohibited; you are responsible for any consequences.**

### Launch the full system

Start the controller node, `robot_state_publisher`, and optionally RViz:

```bash
ros2 launch rebotarm_bringup bringup.launch.py
```

`reBotArmController` connects to real hardware on startup. The default model is
defined by `rebotarm_bringup/config/rebotarm_hardware.yaml` and is currently
`dm`.

If you use the RS arm and have not changed `default_model`, pass `model:=rs`
explicitly and set `channel:=can0`.

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can0
```

Enable RViz to visualize the arm motion:

```bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
```

### Launch only the driver

Unlike `bringup.launch.py`, this starts only the controller node. Use it when
you want the minimal hardware control process without `robot_state_publisher`
or RViz visualization.

```bash
ros2 launch rebotarm_bringup driver.launch.py
```

### Run the controller directly

```bash
ros2 run rebotarmcontroller reBotArmController
```

Unlike `driver.launch.py`, which passes configuration files from
`rebotarm_bringup/config`, running the controller directly falls back to the
default SDK arm configuration. For normal use, launching through ROS is
recommended.

---

## Direct Pose Motion

Without running an example script, you can call ROS services and actions directly.
Start the controller in one terminal:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

In another terminal:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
```

1. Enable the arm:

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger
```

2. Move the TCP to a target pose:

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.0, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, duration: 2.0}"
```

`move_to_pose` executes through the SDK end-pose controller. The arm control
mode is selected by `rebotarm_hardware.yaml`: DM defaults to `posvel`, RS
defaults to `mit`.

3. Close the gripper and return to safe home:

```bash
ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger
```

4. Disable the arm:

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger
```

---

## Example Scripts

All examples assume `reBotArmController` is already running:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM0
```

Examples are installed as ROS2 executables and can be launched with `ros2 run`.

Source files:

```text
src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py
src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py
src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py
src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py
```

### move_to.py

Move all 6 joints to absolute joint angles in radians:

```bash
ros2 run rebotarmcontroller MoveTo -- \
  0.20 -0.20 -0.20 -0.20 0.10 -0.10 \
  --duration 8.0
```

Move a single joint:

```bash
ros2 run rebotarmcontroller MoveTo -- --joint joint3 --position -0.20 --duration 5.0
```

### move_to_pose.py

Move the TCP to a pose:

```bash
ros2 run rebotarmcontroller MoveToPose -- --x 0.30 --y 0.0 --z 0.30 --qw 1.0 --duration 2.0
```

### gravity_compensation.py

```bash
ros2 run rebotarmcontroller GravityCompensation
```

The script calls `/rebotarm/enable`, starts gravity compensation, then on
`Ctrl+C` calls `/rebotarm/gravity_compensation/stop`, `/rebotarm/safe_home` and
`/rebotarm/disable`.

### gripper_control.py

```bash
ros2 run rebotarmcontroller GripperControl
```

Interactive commands:

```text
o / open    open gripper
c / close   close gripper
q / quit    quit
```

---

## API Reference

The full ROS2 API is documented in [API_zh.md](API_zh.md).

It includes:

- Topics, services and actions with their message types
- `/rebotarm` namespace, QoS, units and state machine conventions
- Custom interfaces such as `JointMitCmd`, `JointPosVelCmd`, `ArmStatus` and `MoveToPose`
- Examples for TCP motion, gripper control, gravity compensation and raw commands
- Integration notes for higher-level applications and multi-arm setups

---

## Configuration

`rebotarm_bringup/config/` provides default driver configuration:

| File | Description |
|---|---|
| `rebotarm_hardware.yaml` | ROS2 hardware selection and overrides for DM / RS SDK configs |
| `driver_params.yaml` | ROS parameter example |

Common launch parameters:

| Parameter | Default | Description |
|---|---|---|
| `hardware_config` | built-in `rebotarm_hardware.yaml` | ROS2 hardware config path |
| `model` | empty string | Use `default_model` when empty; set `dm` or `rs` explicitly |
| `channel` | empty string | Use YAML value when empty; override communication channel when set |
| `joint_state_rate` | `100.0` | Publish rate for `/rebotarm/joint_states` |
| `cmd_arbitration` | `reject` | Low-level command arbitration during arm trajectory execution |
| `arm_namespace` | `rebotarm` | ROS namespace prefix |
| `frame_id` | `base_link` | Base frame ID reserved for TF, perception and planning |
| `ee_frame_id` | `end_link` | End-effector frame ID reserved for TF, perception and planning |
| `use_rviz` | `false` | Start bringup RViz |
| `disable_after_safe_home` | `true` | Controls whether motors are disabled after safe home completes |

Model defaults in `rebotarm_hardware.yaml`:

| Model | Default channel | Arm mode | Gripper limits |
|---|---|---|---|
| `dm` | `/dev/ttyACM0` | `posvel` | open `-5.0`, close `0.0` |
| `rs` | `can0` | `mit` | open `5.0`, close `0.0` |

---

## MoveIt 2

MoveIt 2 is the motion planning framework used here for inverse kinematics,
collision checking, trajectory planning and execution. The demos are separated
into their own package so application flows stay isolated from the base driver.
For more details, see the official [MoveIt 2 Documentation](https://moveit.picknik.ai/main/index.html).

MoveIt-related content is split into two packages:

| Package | Purpose |
|---|---|
| `rebotarm_moveit_config` | Robot model, SRDF, kinematics, joint limits, controller and RViz config |
| `rebotarm_moveit_demos` | Application demos based on MoveIt 2 |

The MoveIt environment uses simulated hardware through `ros2_control` and
`move_group` for planning and execution. It is intended for validating the
model, IK, trajectory planning and demo flow in RViz.

This repository also supports real hardware. Before connecting real hardware,
make sure the arm zero configuration, joint directions, joint limits, velocity
limits and gripper range are all correct, or keep the default repository
configuration.

### MoveIt Environment Setup

Make sure the ROS2 environment is available first. You can install packages for
the currently sourced ROS distribution through `ROS_DISTRO`:

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

The MoveIt config and demos are included in this workspace. After installing
dependencies, rebuild the workspace:

```bash
cd your/path/to/rebotarm_ros2
colcon build --symlink-install
source install/setup.bash
```

Verify the MoveIt packages and demo entry points:

```bash
ros2 pkg list | grep rebotarm_moveit
ros2 pkg executables rebotarm_moveit_demos
```

Expected entries include:

```text
rebotarm_moveit_demos draw_square
rebotarm_moveit_demos pick_place
```

### Use MoveIt

MoveIt planning can be used through the RViz GUI or through ROS nodes, in both
simulation and real scenes.

#### Use MoveIt in simulation

MoveIt uses the ros2_control virtual hardware interface for RViz simulation:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_config demo.launch.py
```

For the RS arm, pass `model:=rs` when the default model has not been changed:

```bash
ros2 launch rebotarm_moveit_config demo.launch.py model:=rs
```

By default this starts:

- `move_group`
- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `rebotarm_controller`
- `gripper_controller`
- RViz with the MoveIt MotionPlanning plugin

RViz opens automatically and loads the robot URDF model. You can control motion
through the panel on the left side of the GUI.

#### Use MoveIt with reBotArm hardware

For the real robot, first start the controller with the hardware interface
instead of the virtual controller, then start the hardware MoveIt environment:

```bash
ros2 launch rebotarm_bringup driver.launch.py
```

In another terminal:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_config hardware.launch.py
```

To repeat: before running any demo on real hardware, make sure the workspace is
clear of people and obstacles, verify the planned path in RViz, and be ready to
stop the controller at any time.

### Run the draw-square demo

Start the MoveIt environment first, then run in another terminal:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_demos draw_square.launch.py
```

`draw_square` moves `gripper_tcp` through the four corners of a coplanar rectangle.
Default parameters:

```text
src/rebotarm_moveit_demos/config/draw_square.yaml
src/rebotarm_moveit_demos/config/draw_square_rs.yaml
```

Common parameters:

| Parameter | Description |
|---|---|
| `start_point` | Joint reset position before the demo starts |
| `rectangle_center` | Rectangle center in `base_link` |
| `rectangle_width` / `rectangle_height` | Rectangle dimensions in meters |
| `tcp_rpy` | TCP orientation, defaulting to a downward-facing gripper |
| `tcp_yaw_offsets` | Alternative IK yaw values used to avoid large joint6 wraps |

### Run the pick-place demo

Start the MoveIt environment first, then run in another terminal:

```bash
cd your/path/to/rebotarm_ros2
source install/setup.bash
ros2 launch rebotarm_moveit_demos pick_place.launch.py
```

Default parameters:

```text
src/rebotarm_moveit_demos/config/pick_place.yaml
src/rebotarm_moveit_demos/config/pick_place_rs.yaml
```

Common parameters:

| Parameter | Description |
|---|---|
| `ready_point` | Ready joint position used before and after pick/place |
| `pick_position` | Object bottom-center position in `base_link` |
| `pick_tcp_rpy` / `place_tcp_rpy` | TCP orientation for pick and place |
| `object_dimensions` | Planning-scene object dimensions in meters |
| `max_gripper_width` | Maximum total gripper opening, default `0.09m` |
| `open_gripper_position` / `grasp_gripper_position` / `closed_gripper_position` | Simulated single-side gripper joint positions |
| `hardware_open_gripper_position` / `hardware_closed_gripper_position` | Hardware gripper motor open/close positions |

### MoveIt configuration files

| File | Description |
|---|---|
| `rebotarm_moveit_config/config/rebotarm.urdf.xacro` | Robot model used by MoveIt |
| `rebotarm_moveit_config/config/rebotarm.srdf` | MoveIt groups, end effector and default states |
| `rebotarm_moveit_config/config/rebotarm_rs.urdf.xacro` | RS robot model used by MoveIt |
| `rebotarm_moveit_config/config/rebotarm_rs.srdf` | RS MoveIt groups and collision semantics |
| `rebotarm_moveit_config/config/kinematics.yaml` | IK solver configuration |
| `rebotarm_moveit_config/config/joint_limits.yaml` | Joint limits used by MoveIt planning |
| `rebotarm_moveit_config/config/moveit_controllers.yaml` | Shared DM/RS MoveIt trajectory execution controller config |
| `rebotarm_moveit_config/config/ros2_controllers.yaml` | Shared DM/RS ros2_control controller config |
| `rebotarm_moveit_config/config/initial_positions.yaml` | Initial joint positions for simulated hardware |
| `rebotarm_moveit_demos/config/draw_square.yaml` | Draw-square demo parameters |
| `rebotarm_moveit_demos/config/draw_square_rs.yaml` | RS draw-square demo parameters |
| `rebotarm_moveit_demos/config/pick_place.yaml` | Pick-place demo parameters |
| `rebotarm_moveit_demos/config/pick_place_rs.yaml` | RS pick-place demo parameters |

---

## Demo Projects

The following demos run on both the fake DM driver (`fake_dm_bringup.launch.py`)
and real hardware — the interfaces are identical. Each demo assumes the driver
is already running:

```bash
source /opt/ros/jazzy/setup.bash   # use humble on Ubuntu 22.04
source install/setup.bash
ros2 launch rebotarm_bringup fake_dm_bringup.launch.py   # sim driver + RViz
```

### Greeting (rebotarm_greeting)

Three-position nod greeting. The motion script lives in
`src/rebotarm_greeting/config/greeting.yaml`.

```bash
ros2 launch rebotarm_greeting greeting.launch.py
# trigger:
ros2 topic pub --once /rebotarm/greeting/start std_msgs/msg/Bool "{data: true}"
```

### Dance (rebotarm_dance)

A ~38 s routine: arm-raise intro, two rounds of robotic groove with the TCP
pinned in mid-air, a spatial slide along a rectangle, and a final
point-to-the-sky pose. All joint trajectories are precomputed offline at
30 Hz, so no online IK runs during playback.

```bash
ros2 launch rebotarm_dance dance.launch.py                    # waits for trigger
ros2 launch rebotarm_dance dance.launch.py auto_start:=true   # start immediately
# trigger:
ros2 topic pub --once /rebotarm/dance/start std_msgs/msg/Bool "{data: true}"
```

To adjust the step order, durations or trigger, edit
`src/rebotarm_dance/config/dance.yaml`. To edit the moves themselves, re-run
`precompute_dance.py` and rebuild the package; see
`src/rebotarm_dance/README.md`.

### Gesture recognition (rebotarm_vision)

Camera → MediaPipe Hands (21 keypoints) → rule-based classifier (no model
training) → publishes `/rebotarm/gesture_cmd` (`rock` / `paper` / `heart` /
`none`). Extra dependency:

```bash
pip install --user --break-system-packages "mediapipe==0.10.14" "numpy==1.26.4"
```

`numpy` must stay <2, otherwise it breaks ROS2/pinocchio.

```bash
# standalone tuning tool (no ROS needed), press r to print ratios, q to quit:
python3 src/rebotarm_vision/scripts/test_gesture.py

# ROS node:
ros2 launch rebotarm_vision vision.launch.py                    # with debug window
ros2 launch rebotarm_vision vision.launch.py show_debug:=false  # headless
```

### Gesture actions (rebotarm_gesture_actions)

Plays back a precomputed TCP trajectory when a gesture is recognized:

| Gesture | Trajectory | Motion |
|---|---|---|
| ✊ rock | `rock_push` | TCP push-pull ±6 cm along base X, fast out / slow back ×3 |
| ✋ paper | `paper_figure8` | 26×14 cm figure-8 in the Y-Z plane ×2, with wrist twist |
| 🫶 heart | `heart_draw` | Two-stroke heart, 22 cm tall |

Full pipeline (vision → dispatcher → player), with the driver already running:

```bash
ros2 launch rebotarm_vision vision.launch.py                    # terminal 2
ros2 launch rebotarm_gesture_actions gesture_actions.launch.py  # terminal 3
```

The dispatcher is edge-triggered: holding a gesture fires the motion only
once; lower the hand (recognized as `none`) and raise it again to retrigger.
Without a camera, trigger a motion directly:

```bash
ros2 topic pub --once /rebotarm/action/play std_msgs/msg/String "{data: rock_push}"
```

See `src/rebotarm_gesture_actions/README.md` for how to regenerate the
trajectories.

---

## FAQ / Troubleshooting

### `ros2: command not found`

The ROS2 environment is not available in the current shell, or ROS2 is not
installed. Install ROS2 first, then source the matching distribution once per
terminal session:

```bash
source /opt/ros/humble/setup.bash
```

If you use Jazzy, replace `humble` with `jazzy`.

To load ROS2 automatically in new terminals, add the source command to
`~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### ROS2 executable not found

If `ros2 run` or `ros2 launch` cannot find a package or executable, the
workspace overlay is usually not sourced in the current terminal. Rebuild if
needed, then source the workspace:

```bash
cd your/path/to/rebotarm_ros2
colcon build --symlink-install
source install/setup.bash
```

You can verify installed entry points with:

```bash
ros2 pkg executables rebotarmcontroller
ros2 pkg executables rebotarm_moveit_demos
```

### Serial device not found

If startup reports:

```text
open serial port /dev/ttyACM0 failed: No such file or directory
```

Check the actual device:

```bash
ls /dev/ttyACM*
```

Then override `channel`:

```bash
ros2 launch rebotarm_bringup bringup.launch.py channel:=/dev/ttyACM1
```

For RS, check that the SocketCAN interface is up:

```bash
ip -details link show can0
sudo ip link set can0 up type can bitrate 1000000
ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can0
```

### Permission denied

If the serial device exists but cannot be opened:

```bash
sudo usermod -a -G dialout $USER
```

Log out and log back in for the group change to take effect.

### RViz model is not displayed

Check that URDF mesh paths use:

```text
package://rebotarm_bringup/description/...
```

### FastDDS SHM port warning

If the terminal shows:

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7002: open_and_lock_file failed
```

This is usually caused by stale FastDDS shared-memory lock files after ROS2
processes exited unexpectedly. If services and actions still respond, it usually
does not affect control. To clean it up, stop ROS2 processes first, then run:

```bash
pkill -f ros2
pkill -f reBotArmController
rm -f /dev/shm/fastrtps_port*
```

To temporarily avoid shared-memory transport before launching ROS2:

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

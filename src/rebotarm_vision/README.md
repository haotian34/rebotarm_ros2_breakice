# rebotarm_vision —— 任务三：手势识别

笔记本摄像头 → MediaPipe Hands（21 关键点）→ 纯几何规则分类（不训练模型）
→ 发布 `/rebotarm/gesture_cmd`（std_msgs/String：`rock` / `paper` / `heart` / `none`）。

- ✊ `rock`：食/中/无名/小 4 指全弯（指尖到手腕距离 < 指根到手腕距离 × 阈值；拇指不参与）
- ✋ `paper`：4 指全伸
- 🫶 `heart`：双手比心——两手同时入镜，两手拇指尖相碰、食指尖相碰、手腕分开
- 分类规则：**双手入镜只判 heart/none；单手入镜才判 rock/paper/none**
- 比心额外要求**食指 PIP 关节屈曲角 >45°**（两段指骨夹角，伸直≈0°，比心通常>60°），
  双手布（食指全伸）不会误判为比心；中指/无名指/小指不参与判定（各人比心习惯不同）
- 双手入镜时测试脚本会逐项显示比心诊断（哪个条件不满足一眼可见）
- 去抖：连续 5 帧判定一致才切换输出
- 发布：状态变化立即发布 + 1Hz 心跳重发，`ros2 topic echo` 随时有输出

## 文件

- `rebotarm_vision/gesture_classifier.py`：分类规则本体（纯函数，不依赖 ROS）
- `rebotarm_vision/vision_gesture_node.py`：ROS2 节点封装
- `scripts/test_gesture.py`：独立测试脚本（不进 ROS），用于调节阈值

## 使用

```bash
# 1. 独立测试（调阈值）：画面显示骨架+各指弯曲比值+判定，按 r 打印比值，q 退出
python3 src/rebotarm_vision/scripts/test_gesture.py

# 2. ROS 节点（需先 source /opt/ros/jazzy/setup.bash 和 install/setup.bash）
ros2 launch rebotarm_vision vision.launch.py                 # 带调试窗口
ros2 launch rebotarm_vision vision.launch.py show_debug:=false   # 无显示环境

# 3. 观察输出
ros2 topic echo /rebotarm/gesture_cmd
```

阈值不合适时改 `gesture_classifier.py` 的 `DEFAULT_PARAMS`，
或用节点参数覆盖 `rock_ratio_max` / `paper_ratio_min`。

依赖：`pip install --user --break-system-packages "mediapipe==0.10.14" "numpy==1.26.4"`
（numpy 必须保持 <2，否则破坏 ROS2/pinocchio）

## 手掌位置跟随（DM 机械臂）

复用摄像头和 MediaPipe，单手张开连续 5 帧后，以当前手掌位置与当前末端位置为基准，
跟随左右、上下位移。画面已镜像，画面向右对应基座 -Y，向上对应 +Z；
单目不提供可靠米制深度，因此固定基座 X 方向，末端姿态不锁定。
握拳、双手入镜或手掌消失会暂停，重新张开后从当前位置重新建立基准，不自动回零。

先停止手势动作播放器、语音动作节点、舞蹈和其他向关节发布指令的程序，再启动跟随。
此入口不发布手势动作命令。默认模型为 DM，不适用于直接驱动 RS。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select rebotarm_vision --symlink-install
source install/setup.bash

# 终端一：先用仿真驱动验证（或使用已经启动的 DM 真机驱动）
ros2 launch rebotarm_bringup fake_dm_bringup.launch.py

# 终端二：摄像头 + 手掌跟随
ros2 launch rebotarm_vision palm_follow.launch.py camera_index:=0 show_debug:=true
```

建议先将机械臂置于舒展的悬空姿态，再伸出手掌。全零姿态可能处于奇异位形，
该节点不会自行移动到预备姿态。在线位置雅可比采用阻尼最小二乘法，
检查 URDF 关节限位，但不做环境/自碰撞规划。

默认限制：关节 0.3 rad/s、末端目标步进 0.04 m/s、相对起点最大目标位移 0.12 m。
手掌或任一关节反馈超过 0.25 秒未更新就停止发送新指令；驱动可能完成最后一个小步。
若实际关节与指令偏差超过 0.12 rad，则重新等待手掌获取。

参数可通过单独启动控制节点调整（同时另开视觉节点，设置 `publish_gestures:=false`）：

```bash
ros2 run rebotarm_vision palm_follow_node --ros-args \
  -p max_joint_speed:=0.2 -p max_tcp_speed:=0.03 \
  -p horizontal_span:=0.20 -p vertical_span:=0.16 -p max_displacement:=0.10

# 无硬件验证：运动学方向、限速/范围、失效暂停、重新获取
/usr/bin/python3 src/rebotarm_vision/test/check_palm_follow.py
```

`/rebotarm/palm_position` 为 `geometry_msgs/PointStamped`：
`frame_id=camera_normalized`，x/y 为镜像图像中的归一化掌心坐标；
z=1 表示单手张开有效，z=0 表示失效，**z 不是深度**。
控制节点依赖 ROS 环境提供的 `pinocchio` 和 numpy，使用系统 `/usr/bin/python3`，
避免 Conda Python 与 ROS 的 Python 版本冲突。

# rebotarm_vision —— 任务三：手势识别

笔记本摄像头 → MediaPipe Hands（21 关键点）→ 纯几何规则分类（不训练模型）
→ 发布 `/rebotarm/gesture_cmd`（std_msgs/String：`rock` / `paper` / `heart` / `none`）。

- ✊ `rock`：食/中/无名/小 4 指全弯（指尖到手腕距离 < 指根到手腕距离 × 阈值；拇指不参与）
- ✋ `paper`：4 指全伸
- 🫶 `heart`：双手比心——两手同时入镜，两手拇指尖相碰、食指尖相碰、手腕分开
- 分类规则：**双手入镜只判 heart/none；单手入镜才判 rock/paper/none**
- 比心额外要求**两食指朝内汇合**，允许相机投影中的食指接近直线，
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

复用摄像头和MediaPipe，单手张开3帧后开始跟随。
**摄像头装在机械臂上（眼在手上）**，因此采用速度伺服：手偏离画面中心越远，
末端向该方向移动越快，直到手回到画面中心（约 4% 死区内停止）。
画面已镜像（自拍视角）：手往用户右侧偏 → 基座 +Y，偏上 → +Z；
单目不提供可靠米制深度，因此固定基座 X 方向，末端姿态不锁定。

行为语义（`palm_position` 消息 z 字段）：
- `z=1.0` 单手张开：正常跟随
- `z=0.5` 手在画面中但未张开（握拳/双手）：暂停保持，基准保留
- `z=0.0` 画面中无手：**追丢模式**——沿手最后偏离的方向以 60% 速度继续追，
  最多 2.5 秒，随后原地保持；重新张手即刻恢复跟随

安全边界：以起手点为原点的盒形工作空间（左右 ±0.25m、上 +0.18m、下 -0.12m，
X 固定），到达边界即停；指令与实物偏差 >0.12 rad 自动等待重新张手。

**复位**：演示中随时可让手臂平滑回到起手点（约 2 秒）并等待重新张手：
`ros2 topic pub --once /rebotarm/palm_follow/reset std_msgs/msg/Bool "{data: true}"`

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

节点启动后自动用 3 秒平滑移动到「鸡脖子」起手位（home_pose），
到位后张手即可开始跟随，无需手动预置姿态。工作空间以起手位为固定基准
（左右 ±0.25m、上 +0.18m、下 -0.12m），与从哪里张手无关。在线位置雅可比采用阻尼最小二乘法，
检查 URDF 关节限位，但不做环境/自碰撞规划。

默认高响应配置：控制100Hz，驱动批量透传`pos_vel_rate=100`Hz，指令间隔10ms；关节1~3上限5rad/s、
关节4~6上限3rad/s（DM SDK配置），末端目标限速0.6m/s，跟随增益1.65，平滑系数0.85。
手偏离中心10%时，扣除2%死区后的目标速度约0.132m/s，仍受反馈、制动和速度上限约束。
正常关节加速度上限12rad/s²、末端加速度上限2m/s²；反向制动倍率`braking_gain=1.5`，
减少加速后转向拖延。图像误差直接生成连续速度，
不再积累位置目标或将位置误差除以控制周期放大；指令姿态落后反馈时连续降速，
在原0.12rad硬停止阈值前减轻追赶；仅扩大误差的运动触发降速，反向纠偏和切向运动不被旧误差拖慢。
`max_target_lead=0.06`是沿当前运动方向相对反馈的笛卡尔降速阈值。
每关节下发的电机限速匹配当前指令速度，避免每个小采样点快速到位后停等。
启动归位仍为3秒、限速0.8rad/s；手动复位限速0.18m/s，追丢增益系数约0.109，保持原追丢速度。
工作空间及反馈误差保护不变，暂停/视觉过期会清除积累目标；实际指令姿态也进行FK工作空间检查。
跟随launch启用轻量MediaPipe模型，独立采集线程只保留最新帧；以60Hz检查新帧，
没有新帧就返回（不会提高摄像头硬件帧率或重复推理）；时间戳在推理前生成，避免旧图像被当作新观测。
已张开的单手短暂分类不确定时允许0.1秒过渡，明确握拳/双手立即暂停，不确定状态不能首次激活跟随。
这些是软件设置，不代表已经验证机械臂带负载的最大性能。可用
`startfollow max_joint_speed:=0.8 max_tcp_speed:=0.18 follow_gain:=0.3 smoothing:=0.5 chase_gain_scale:=0.6`
恢复上一档速度。

修改为100Hz后需同时重启实机驱动和跟随节点。驱动发送定时器串行执行，
只保留最新目标，不积压过时指令；常规状态读取由原定时器负责，模式变化仍立即发布。
若串口负载高，可用`startreal pos_vel_rate:=50.0`和`startfollow rate_hz:=50.0`恢复20ms间隔。
控制频率不等于摄像头帧率，视觉仍按实际新帧更新。
手掌或任一关节反馈超过0.4秒未更新就停止更新目标／等待重新获取；驱动可能完成最后一个小步。
若实际关节与指令偏差超过 0.12 rad，则重新等待手掌获取。

参数可通过单独启动控制节点调整（同时另开视觉节点，设置 `publish_gestures:=false`）：

```bash
ros2 run rebotarm_vision palm_follow_node --ros-args \
  -p max_joint_speed:=0.2 -p max_tcp_speed:=0.03 \
  -p follow_gain:=0.10 -p max_displacement:=0.20

# 无硬件验证：运动学方向、限速/范围、失效暂停、重新获取
/usr/bin/python3 src/rebotarm_vision/test/check_palm_follow.py
```

`/rebotarm/palm_position` 为 `geometry_msgs/PointStamped`：
`frame_id=camera_normalized`，x/y 为镜像图像中的归一化掌心坐标；
z=1 表示单手张开有效，z=0 表示失效，**z 不是深度**。
控制节点依赖 ROS 环境提供的 `pinocchio` 和 numpy，使用系统 `/usr/bin/python3`，
避免 Conda Python 与 ROS 的 Python 版本冲突。


## 真机与仿真的摄像头选择

从项目根目录加载快捷命令后使用：

```bash
source ./alias.txt
startvision
```

快捷命令通过 ROS 节点发现识别 `reBotArmController`（真机）和 Fake DM/RS 驱动（仿真），
可跨终端使用。真机模式查找 RealSense 的彩色视频节点；仿真模式查找其他 USB/UVC
彩色摄像头（本机为笔记本摄像头）。没有驱动时默认仿真模式。
若两种驱动同时存在，需显式选择模式；自动检测不限命名空间，多个机械臂请显式指定。
这里的 sim 指使用电脑摄像头，并非 MuJoCo 渲染图像。

```bash
startvision mode:=real
startvision mode:=sim
# 显式指定摄像头，优先于自动选择：
startvision camera_device:=/dev/v4l/by-id/实际摄像头节点
# 保留旧版编号用法：
startvision camera_index:=0
```

自动选择通过 V4L2 格式区分彩色和深度/红外/元数据节点，不固定 video 编号。
发现零个或多个候选时会报错，需检查设备连接、权限并明确选择，不会将真机摄像头缺失
静默替换为电脑摄像头。RealSense 必须通过 USB 接到电脑；机械臂串口不传输图像。
如果机械臂摄像头不是 RealSense，或电脑有多个外接摄像头，在各电脑自己的 shell 配置设置：

```bash
export REBOTARM_REAL_CAMERA='/dev/v4l/by-id/机械臂彩色摄像头节点'
export REBOTARM_SIM_CAMERA='/dev/v4l/by-id/电脑彩色摄像头节点'
```

启动日志会显示 `Vision camera mode` 和 `Camera source`，用于确认所选设备。
`startvison` 也作为 `startvision` 的拼写兼容命令提供。
直接 `ros2 launch rebotarm_vision vision.launch.py` 仍兼容旧行为（`camera_index:=0`），
可用 `camera_mode:=real` 或 `camera_mode:=sim` 启用设备自动选择。
本次修改针对 `vision.launch.py`；单独运行手掌跟随 launch 仍使用该 launch 的摄像头参数。


### 双手大爱心识别与诊断

比心使用两手拇指尖靠近、食指尖靠近、手腕分开、每手拇指与食指张开、两食指向内汇合
的几何规则。食指投影可以接近直线，不再要求两手食指都屈曲 45 度。
窗口显示 `hands` 数量，以及各项比心条件的 `OK/FAIL`、测量值和阈值。
其中距离按平均手掌尺度归一化；诊断和实际分类使用同一组检查。
仍需连续 `debounce_frames` 帧一致，单手小爱心不在本规则覆盖范围内。

`vision.launch.py` 现在可传入全部分类阈值，例如
`startvision heart_index_inward_min:=0.15`。调试识别时先停止动作播放器，
只运行视觉节点，避免手型调整时误触发动作。修改后重启视觉节点即可加载新代码。

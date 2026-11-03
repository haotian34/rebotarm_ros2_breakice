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

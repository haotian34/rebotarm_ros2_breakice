# 本地 Whisper 语音动作控制（DM）

链路：麦克风 → 本地 Whisper（faster-whisper）转写 → 固定口令 → `/rebotarm/action/play`
→ `rebotarm_gesture_actions` 轨迹播放器 → 驱动。无需 API Key、不上传音频、没有 API 调用费用。

本模块复用 **DM 专用轨迹和关节限位**，不适用于 RS；仅更改命名空间不能适配 RS。
轨迹播放器的动作包括平滑接入、轨迹回放以及回到全零姿态。

## 安装和构建

在已经能构建运行本项目的 ROS 2 环境中执行：

```bash
sudo apt install libportaudio2
python3 -m pip install --user --break-system-packages 'faster-whisper>=1.1,<2' 'sounddevice>=0.4' numpy
source /opt/ros/jazzy/setup.bash  # Humble 用户改为 humble
colcon build --symlink-install --packages-select rebotarm_voice rebotarm_gesture_actions
source install/setup.bash
# 首次联网下载 base 多语言模型（约 150 MB），之后可断网运行
ros2 run rebotarm_voice download_model --model base
```

若终端默认进入 Conda，请先 `conda deactivate`，确认 `python3` 与 ROS 使用同一版本
（本机 Jazzy 为 `/usr/bin/python3`，Python 3.12），再安装依赖及构建。
Python 依赖应安装到运行 ROS 节点的同一 Python 环境中；rosdep 不会安装这里的
faster-whisper 和 sounddevice。
模型默认保存在 Hugging Face 用户缓存中；运行节点使用 `local_files_only=True`，
不会自动下载模型。录音和识别均在本机完成，不需要 `OPENAI_API_KEY`。

## 先在 DM 仿真中使用

终端一（先完成项目原有驱动包的构建）：

```bash
source install/setup.bash
ros2 launch rebotarm_bringup fake_dm_bringup.launch.py
```

终端二（加载环境）：

```bash
ros2 launch rebotarm_voice voice_control.launch.py
```

等待终端出现 `READY: local Whisper loaded` 后再触发录音。
默认 `execute_motion:=false`，识别后仅报告 `DRY_RUN:<轨迹名>`。
确认识别效果后，重启终端二中的 launch 开启执行：

```bash
ros2 launch rebotarm_voice voice_control.launch.py execute_motion:=true
```

终端三触发录音，然后在 4 秒录音窗口内说出口令：

```bash
ros2 service call /rebotarm/voice/listen std_srvs/srv/Trigger '{}'
```

服务立即返回“开始录音”，并非动作执行完成。查看结果：

```bash
ros2 topic echo /rebotarm/voice/transcript
ros2 topic echo /rebotarm/voice/status
```

| 完整口令 | 轨迹 |
|---|---|
| 出拳 | rock_push |
| 请画八字 / 画8字 | paper_figure8 |
| 画爱心 / 画心 | heart_draw |
| 画圆 / 画个圆 | paper_circle |

空白和标点可省略。直接说命令即可，无需唤醒前缀；命令前可加“请”，其他附加内容、否定句和组合指令均拒绝。
这不是自由对话控制器，也没有语音急停功能；语音识别受录音时长和本地推理速度影响。
真机需先按原项目流程准备并使能驱动，使用硬件急停保障现场操作。

## 配置和调度

通过 `config:=/绝对路径/voice.yaml` 加载自定义参数文件（节点名 `rebotarm_voice`）。
默认文件位于 `config/voice.yaml`。可设置录音时长（0~15 秒）、麦克风名称或索引
（`audio_device` 为字符串）、静音 RMS 阈值、语言和唤醒词。采样率固定为 16000 Hz。
麦克风列表：`python3 -m sounddevice`。默认 `wake_word: ''`，直接说命令；如需前缀可自行配置 `wake_word`。

默认 launch 启动一个播放器；若已经运行 gesture_actions，请使用 `start_player:=false`，
保证只有一个播放器。执行模式要求新鲜的 IDLE 心跳和唯一的 play 订阅者；
忙碌、冷却和录音过程中不接受重复请求，识别完成时再次检查状态。
指令超过 `max_command_age` 后丢弃，发送后等待 BUSY 回执，不自动重试。
`REQUESTED` 只表示已经发布请求；实际完成状态查看 `/rebotarm/action/status`。

请不要同时运行舞蹈、招手、MoveIt 等其他运动指令源；现有低层话题之间没有全局调度锁。
状态心跳只负责播放器可用性检查，不代表电机已经使能或实际到达轨迹目标。

### 本地模型配置

默认 `model_size: base`、`model_device: cpu`、`compute_type: int8`、`cpu_threads: 4`，
无需 CUDA。模型只加载一次，后续每次录音复用。

需要尝试更大模型时，先执行 `ros2 run rebotarm_voice download_model --model small`，
再将自定义 YAML 中的 `model_size` 改为 `small`。更大模型通常需要更多内存和推理时间。
也可以将 `model_size` 设为已经下载好的 CTranslate2 Whisper 模型目录。
`model_cache` 可配置缓存目录；下载时需对应传入 `--cache-dir /绝对路径`。
模型缺失时节点报告加载失败，不接受录音；先下载并重启节点。

推理参数 `model_device: cuda` 和 `compute_type: float16` 适用于已配置兼容
CUDA/cuDNN 的 NVIDIA 环境，默认 CPU 模式不依赖这些库。

实现参考：https://github.com/SYSTRAN/faster-whisper

## 开发验证

```bash
PYTHONPATH=src/rebotarm_voice python3 -m pytest src/rebotarm_voice/test -q
```

单元测试使用模拟音频和本地模型，覆盖口令拒绝、离线模型加载、音频归一化和静音过滤。

## RViz 底部红色叠加字幕

语音 launch 默认启动 `rviz_subtitles`，将识别结果和状态发布到
`/rebotarm/voice/subtitles`（`rviz_2d_overlay_msgs/msg/OverlayText`）。
字幕直接叠加在机械臂三维画面底部居中，字体为红色、背景透明，不占用左侧面板。

需要 `rviz_2d_overlay_msgs` 和 `rviz_2d_overlay_plugins`。本机已在工作空间构建。
新机器可通过 apt 安装（Jazzy 示例）：

```bash
sudo apt-get install ros-jazzy-rviz-2d-overlay-msgs ros-jazzy-rviz-2d-overlay-plugins fonts-noto-cjk
```

也可以像本机一样从固定版本源码构建：

```bash
git clone --depth 1 --branch 1.4.2 https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins.git third_party/rviz_2d_overlay_plugins
PATH=/usr/bin:/bin colcon build --symlink-install --packages-select rviz_2d_overlay_msgs rviz_2d_overlay_plugins --cmake-args -DCMAKE_DISABLE_FIND_PACKAGE_Qt6=TRUE -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_IGNORE_PREFIX_PATH=/home/syh/anaconda3
source install/setup.bash
colcon build --symlink-install --packages-select rebotarm_voice rebotarm_bringup
source install/setup.bash
```

`third_party` 不纳入本仓库版本控制，换电脑需安装或下载此依赖。
关闭旧 RViz，重新运行 `fake_dm_bringup.launch.py`，默认配置包含 **Voice Subtitles**。
自定义 RViz 配置可 Add → `rviz_2d_overlay_plugins/TextOverlay`，Topic 选择
`/rebotarm/voice/subtitles`，三个 `Overtake ... Properties` 保持 false，让消息控制样式和位置。
旧版 Image 面板应删除；当前话题类型已由 Image 改为 OverlayText。

若语音 launch 没有启动字幕节点，可单独执行：

```bash
ros2 run rebotarm_voice rviz_subtitles
```

识别结果出来后立即更新；目前仍为 4 秒录音结束后转写，不是逐字流式转写。
开始录音时清除上一段文字，显示录音状态。非默认命名空间需对应修改 Topic。
`show_subtitles:=false` 可关闭自动字幕节点。默认字幕框宽 640 像素、字号 22 pt，
可单独启动节点并通过 ROS 参数 `subtitle_width`、`subtitle_text_size` 调整。

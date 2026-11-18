# reBot Arm Studio

项目的中文网页控制台，机械臂型号固定为 **B601 DM**。默认在网页内显示真正的 RViz 2 窗口，直接订阅本机 ROS 2 的关节状态与 TF，支持 DM 仿真、网页一键连接真机和已有真机驱动。可切换到使用真实 URDF / STL 的浏览器 3D 演示。前端和 noVNC 资源已包含在仓库，无需 Node、npm 或 CDN。

## 日常使用：打开网页自动启动本机仿真

本机安装好入口后，直接打开 **http://127.0.0.1:8088**，或双击桌面/应用菜单中的 **reBot Arm 仿真工作台**。页面自动启动项目的 DM Fake 驱动、连接 ROS 2 关节与状态反馈，并自动启动原生 RViz 的独立显示会话、开放仿真关节和夹爪控制，不需要打开命令行。

网页顶部显示启动进度，并提供“停止仿真”“启动仿真”和“仅看网页演示”。刷新、多标签页打开及重复点击不会重复启动节点。停止后可在网页重新启动；关闭标签页不会停止共享仿真。

实现方式是一个随用户登录启动的轻量网页服务：服务待机时不初始化 ROS，页面通过同源 POST 请求启动 Fake 会话。仿真使用独立 ROS 域 **187**、命名空间 **/rebotarm_web** 和独立 ROS Context，不连接串口、CAN 或真机，不自动启动摄像头或麦克风。它运行 Fake 关节仿真，不包含 MuJoCo 物理仿真、MoveIt 或视觉识别节点。

本次已在当前电脑配置好服务和快捷方式。迁移到其他 Linux 电脑时，在已构建的工作空间中进行一次安装：

```bash
/usr/bin/python3 web/setup_rviz.py
/usr/bin/python3 web/install_desktop.py
```

安装范围仅为当前用户：`~/.config/systemd/user/rebotarm-studio.service`、应用菜单及桌面快捷方式，不需要 sudo。启动脚本自动加载 ROS 2 和工作空间环境，使用系统 Python。源码更新涉及 ROS 包时应先重新构建相应包。

可选卸载：`/usr/bin/python3 web/install_desktop.py --remove`。停止/移除的是本机仿真服务，不影响原有真机启动脚本。项目目录移动后需重新安装入口。

本项目按本地运行维护，不需要云服务器、在线网站或外网连接。日常使用本机地址和桌面快捷方式。

## 真机连接：复用仿真的全部操作页面

进入左侧“真机连接”，选择自动识别的 USB 串口，点击“一键连接真机”。只有此操作会启动原项目的 `rebotarm_bringup/driver.launch.py model:=dm`；沿用原硬件配置、SDK、控制器与 ROS 服务，不修改原仿真、实机或动作节点。默认独立域 188、命名空间 `/rebotarm_web_hardware`，与仿真互斥。驱动按原逻辑使能并保持当前姿态，收到六轴和控制器反馈后才显示已连接。

连接后，工作空间、动作实验室、视觉与交互及原生 RViz 统一使用所选真机。关节滑块、夹爪、动作与倍率、手势、手掌跟随、语音和文字继续复用现有接口。连接页显示实际关节角、速度、力矩、电机状态码、控制模式、使能状态及反馈年龄，不以设备存在或浏览器动画代替成功连接。

“停止并断开”先调用原 `safety_stop`，再关闭本页启动的原驱动；未能确认停止时显示错误，不触发原驱动退出时的自动回位。“切换到仿真”在停止真机后恢复原 Fake 会话。切换时先关闭旧对象的视觉进程及 RViz，阻止新运动，再接入新对象。不会终止外部启动的驱动；串口被占用时显示占用状态。拔线或反馈中断后停止本页驱动，不自动重连或切回仿真。页面刷新及重复点击均不重新连接真机。

串口列表只检查设备、账户权限和占用，不打开设备，不执行 sudo 或更改系统权限。账户已有设备组资格但用户服务的组信息未刷新时，使用 `sg` 以现有组权限启动原驱动。

## 网页内的原生 RViz

本机工作台默认选择“原生 RViz”，支持直接操作 RViz 的相机、菜单、显示面板，以及全屏和独立窗口。右侧关节滑块仍通过原有 ROS 控制接口执行；拖动只设目标，松开后按所选速度运动。RViz 使用项目原有的配色 URDF、网格及字幕显示配置。网页不再通过 HTTP 状态轮询驱动 RViz 画面，robot_state_publisher 直接订阅驱动的 JointState，TF 发布上限 50 Hz，RViz 目标渲染频率 30 FPS；实际显示帧率取决于本机渲染与传输性能。

实现为 RViz 2 → 专用 Xvfb 显示 → x11vnc / WebSocket → 同源 noVNC iframe。只转发专用 RViz 显示，不共享用户桌面。X11 使用随机认证，VNC 使用每次启动生成的临时密码，显示端口只监听 127.0.0.1，并校验网页 Origin。密码通过受 Host / Origin 保护的同源 API 获取，不放进 URL 或日志。可视化的 robot_description、tf、tf_static 使用 `/rebotarm_web_rviz` 专用话题，不覆盖外部系统 TF。只读桥接默认不加载项目的 Arm Safety 控制面板；允许运动时使用对应机械臂命名空间。

`web/setup_rviz.py` 从 Ubuntu 官方软件包源下载 Xvfb、x11vnc、noVNC、websockify 及相关库，解包到项目的 `web/.rviz-runtime`，不修改系统软件。需要已有 ROS 2、RViz、robot_state_publisher、项目 RViz 插件，以及 Ubuntu 桌面的 xauth / xwininfo。当前电脑已安装这些网页依赖。`.rviz-runtime` 不进入 Git；迁移机器后运行一次安装脚本即可。

刷新和多标签页共享一个 RViz 实例；关闭标签页只断开显示，停止仿真会同时停止 RViz。服务退出清理自己创建的显示进程。显示断开时可点击“重新连接”。切换“网页 3D 演示”后按需加载 Three.js / STL，并暂停隐藏的浏览器渲染。

网页、ROS、RViz 和显示通道都运行在本机。所有日常运行所需的网页资源均已放在项目中，启动不访问在线入口。

## 手动启动（开发用）

在仓库根目录运行：

```bash
python3 web/server.py
# 浏览器打开 http://127.0.0.1:8088
```

浏览器演示不连接 ROS 或硬件。包含三维工作空间、六轴与夹爪预览、动作实验室、视觉与交互、系统拓扑、日志搜索/导出与接入指南。网页动作读取原项目动作剧本和逐帧轨迹，**不执行 IK、碰撞检测或物理仿真**。

## 手动控制：松开滑块后按速度移动

六个关节滑块都采用相同操作：拖动期间只修改目标，不发送运动命令；松开后等待约 180 ms 合并重复事件，再按所选速度上限移动。键盘连续调节结束后同样提交最终目标；数字框可按 Enter 或结束编辑提交。每行显示目标角度和实际反馈。

“关节速度上限”可选 0.1 / 0.3 / 0.5 rad/s，运动时间按当前位置到目标的角度差自动计算，并为平滑加减速留余量。大角度、慢速度的移动自动增加时长，手动控制不使用固定 2 / 4 / 6 秒。浏览器演示和本机 DM Fake 仿真均无需硬件；DM 真机的单关节调节复用原 `JointPosVelCmd` 接口：只发送该关节的最终位置和电机速度上限一次，由原驱动及电机执行，避免网页轨迹插值与底层控制叠加。浏览器轮询仅用于显示和到位确认，不连续修改目标。其他标准轨迹请求保留 FollowJointTrajectory 接口和原有权限检查。

一次拖动只改变所选关节，其他关节从当前反馈保持。ROS 手动目标通过 `move_joint` 异步提交，长时间慢速移动不会占住 HTTP 请求；新目标排队，同一关节尚未执行的目标仅保留最新值。软件停止/失能清空队列并取消当前运动。停止、切换模式和取消拖动均清理待发送的滑块事件。

新增请求：`POST /api/command`，JSON 为 `{"command":"move_joint","joint":"joint2","position":-0.5,"speed":0.3}`。位置单位 rad，速度上限 0.05–0.5 rad/s；服务端按 URDF 校验限位、实时状态与使能状态。接口立即返回 accepted 和 sequence，实际进度通过 `/api/state` 的 `manual_motion` 查询，区分已接收、运动中和实际完成。

Fake 驱动现按每个 JointPosVelCmd 的 vlim 限制关节速度；零或未指定 vlim 保留原全局速度上限。原 `move_joints` 固定时长接口继续保留，供既有调用者使用。

## 动作按钮：网页预览、仿真与真机

网页不再定义另一套舞蹈、出拳或爱心姿态。连接 ROS 时，网页在相同 ROS 域和命名空间内启动**原项目未修改的 ROS 节点**，触发原始 start / action/play 话题，并等待原节点的完成状态与实际反馈。RViz 直接呈现实际机器人状态；网页 3D 为同一反馈的可选显示，不绘制运动轨迹线。

| 网页按钮 | 对应命令 / 原节点 | 原始配置或轨迹 |
|---|---|---|
| 三位置问候 | greeting_node | greeting.yaml 全剧本 |
| 机械臂舞蹈 | dance | dance.yaml + groove 两圈 + slide，track_speed=3.0（launch 默认值） |
| 出拳 / 石头 | punch | rock_push |
| 画八字 / 布 | figure8 | paper_figure8 |
| 画爱心 / 比心 | heart | heart_draw |
| 画圆 | circle | paper_circle |

删除了先前冒充原动作的简化关节预设与取放按钮；MoveIt 规划仍由原项目节点承担。问候和舞蹈保留所有夹爪、停顿、挥手、造型与回零步骤，手势轨迹保留平滑接入和默认回零。

动作倍率为 0.25× / 0.50× / 0.75× / 1.00×，1.00× 使用命令行默认节奏。只给本次网页启动的子进程传入倍率参数及临时配置副本：采样频率与速度上限同比缩放，移动、停顿、斜坡与夹爪节奏反向缩放，不跳帧、不改轨迹。原包节点、YAML 与从命令行启动的行为不变。Fake 的夹爪电机角方向由临时配置适配，DM 真机保持原负角度。网页托管 Fake 支持原动作的速度上限，手动滑块仍受独立的 0.5 rad/s 上限限制。

启动前校验完整动作、DM 限位与参数；只接受固定动作名和倍率，不接收脚本或自定义轨迹。如果发现同命名空间已有外部关节控制发布器，则拒绝启动，防止与命令行动作争抢控制。只停止网页本次拥有的动作进程，不关闭用户的 RViz、ROS daemon 或原有节点。停止按钮调用原 safety_stop，锁止关节与夹爪并结束本次动作；失能、反馈过期和故障也会中止，解除后不会自动续播。完成时同时确认节点完成状态及实际关节/夹爪反馈。

接口：`POST /api/command`，`{"command":"run_action","action":"dance","speed":1.0}`；停止为 `{"command":"stop_action"}`。`action_motion` 提供执行状态、阶段和进度。保持只读、使能、故障与新鲜反馈检查，不自动启动或使能真机。刷新不重新提交动作。

无 ROS 连接的网页预览通过只读 `POST /api/actions/preview` 获取原动作采样。适配器仅读取固定本地节点中的纯运动方法，直接复用原公式及 YAML；不会启动 ROS 节点或发送运动指令。服务器需系统 PyYAML（ROS 环境自带）。

## 网页 3D 性能

本机网页可直接用 `http://127.0.0.1:8088/?view=browser` 打开 Three.js 视图。默认入口仍显示原生 RViz。两种显示均可连接 Fake 仿真，不要求真机。

ROS 3D 显示使用 `GET /api/joints/stream` 持续读取最新 JointState 快照，最多 30 次/秒；HTTP 状态轮询保留给控制器状态和命令进度，不再决定模型姿态刷新率。浏览器将已收到的真实反馈缓冲约 80 ms，在相邻反馈之间逐帧插值，断流后保持最后位置，不向未来外推或生成控制命令。切换 RViz、断开或关闭页面时关闭流，流失效时保留低频状态回退。

浏览器加载 `dm.web.json` 和 `optimized/` 下的专用显示网格。DM 从 430,912 降为 104,016 个三角面，下载量约减少 75%。原 URDF、原 STL、材料分组、关节原点、限位和 RViz 资源不变。网页使用较轻的光照材质、接触阴影，关闭实时阴影贴图，将像素倍率限制到 1.25。已移除运动轨迹划线；静止或不可见时跳过绘制，动作演示的滑块和文字最多每 100 ms 更新。画面右上角显示实际绘制 FPS，静止时显示“画面静止”。

更新 URDF / 网格后，先运行 `export_models.py`，再用 Node 运行 `web/optimize_models.mjs`，生成网页显示模型。简化工具为离线的 meshoptimizer v0.23，误差阈值为每个网格尺寸的 0.15%，保留边界；运行网站不依赖 Node、WASM 简化器或外网。许可见 `tools/vendor/MESHOPT-LICENSE.md`。

## ROS 2 接入

加载环境后使用系统 Python（Conda Python 通常无法导入系统 ROS 2）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 web/server.py --ros
```

从本地地址打开网页后自动连接同源 API。默认只读，不会启动驱动、自动使能或发送运动命令。

先用项目 Fake 驱动验证。两个终端均需加载以上 ROS 环境：

```bash
# 终端 1：无真机、无 RViz 的 DM 仿真
ros2 launch rebotarm_bringup fake_dm_bringup.launch.py use_rviz:=false

# 终端 2：允许网页控制这套仿真
/usr/bin/python3 web/server.py --ros --allow-motion
```

真实硬件控制前检查机械臂现场、零点、方向、限位和底层 SDK 配置。网页的“软件停止”调用 `/safety_stop`，不是硬件急停。安全锁止须按项目原有的现场恢复流程解除；网页不自动解除锁止或重试运动。

### 接口与保护

- `GET /api/rviz/status`：查看原生显示状态，不返回认证信息；`GET /api/rviz/session` 供同源 noVNC 获取临时会话。
- `POST /api/rviz/start`、`POST /api/rviz/stop`：仅接受空 JSON 对象，只管理专用显示进程，不启动真机驱动。
- `GET /api/runtime`：查看当前仿真或真机的 stopped / starting / running / stopping / error 状态。
- `POST /api/runtime/start`、`POST /api/runtime/stop`：仅在本机管理模式可用，仅接受空 JSON 对象。start 只启动仿真；真机必须通过真机连接页显式连接。
- `GET /api/hardware`：只读设备、连接进度和实际反馈，不启动节点或打开串口。
- `POST /api/hardware/connect`：仅接受已发现的 `device` 串口；`disconnect`、`simulation` 仅接受空 JSON。
- `GET /api/joints/stream`：只读 SSE 关节反馈，最多 30 Hz，包含单调采样时间；未连接 ROS 返回 409，沿用 Host / Origin 检查，不启动任何节点。
- `GET /api/state`：实际 JointState、ArmStatus、夹爪、手势和语音字幕；含反馈年龄，缺失数据返回 null。
- `POST /api/command`：仅接受固定白名单 `enable`、`disable`、`safe_home`、`safety_stop`、`move_joints`、`move_joint`、`gripper`、`run_action`、`stop_action`。
- `move_joints` 的 `positions` 必须为六个 rad 数值；按对应 URDF 校验限位。`duration` 为 2–30 s。
- DM POS_VEL 单关节控制使用原 `JointPosVelCmd` Topic，仅向选中关节发送一次目标与速度上限；等待三次新的低速到位反馈，超时或反馈中断调用原 safety_stop。其他轨迹请求沿用 FollowJointTrajectory；Fake 使用原插值和 Topic 接口。
- `LOWLEVEL_STREAMING` 会按原逻辑关闭末端控制循环，这不代表电机控制失效；保留使能、实时反馈、故障与锁止检查。
- DM 真机夹爪使用 -5..0 rad；项目 Fake 驱动使用 0..5 rad。网页统一显示 0–90 mm。
- 运动要求关节反馈不超过 1.5 s、状态不超过 2 s，且控制器使能、控制循环运行、无故障/安全锁止/重力补偿。缺失或过期立即拒绝。
- 同时仅运行一个运动命令；软件停止与失能不等待运动锁。真实轨迹平均角速度限制为 0.5 rad/s，Fake 平滑插值峰值限制约 0.5 rad/s。
- 软件停止在只读桥接下也可调用。服务退出会清理本页拥有的驱动连接；外部手动接入的驱动仍由原有流程管理。
- 服务只监听 127.0.0.1；校验 Host、Origin、JSON 类型、长度、有限数值与关节限位，不执行 shell。

## 本地运行与连接

网页服务与显示通道只监听 127.0.0.1，不需要公网地址。使用本机同源页面控制 ROS。若开发中确需指定其他来源，启动时可用 `--allow-origin https://具体站点域名` 显式加入来源白名单。

摄像头和麦克风只在点击功能后申请权限。画面和录音通过本机桥接交给原视觉节点及本地 Whisper；识别画面与 RViz 同时展示。MoveIt 与 MuJoCo 仍由原项目入口管理。

## 更新模型

```bash
python3 web/export_models.py
```

该脚本从 `src/rebotarm_bringup/description/DM` 导出所有被 URDF visual 引用的网格、材质、连接层级及限位到 `dist/assets/models`。请更新 URDF 后重新导出。网页关节正运动学使用 URDF 的 xyz/rpy/axis；DM 的末端 frame 为 `end_link`，未额外推导指尖 TCP。

## 验证

```bash
python3 -m unittest discover -s web/tests -v
node --test web/tests/test_manual_control.mjs web/tests/test_joint_playback.mjs web/tests/test_web_models.mjs web/tests/test_action_presets.mjs
node --experimental-vm-modules web/tests/test_app_startup.mjs

# 可选：真实 ROS 2 接口集成测试，只启动 Fake，不访问硬件
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 web/tests/integration_ros.py
/usr/bin/python3 web/tests/integration_managed.py
/usr/bin/python3 web/tests/integration_manual.py
/usr/bin/python3 web/tests/integration_rviz.py
/usr/bin/python3 web/tests/integration_stream.py
/usr/bin/python3 web/tests/integration_actions.py
/usr/bin/python3 web/tests/integration_action_contract.py
```

ROS 集成测试分别使用隔离域 186 / 188 / 190 / 191 / 193 / 195 / 196 和本地发现，覆盖真实订阅、只读拒绝、Fake 关节运动、夹爪、安全锁止与失能。RViz 测试覆盖 DM 原生窗口、实时隔离 TF、关节运动、Origin 拒绝和进程清理，不访问硬件。动作测试覆盖六组原版动作、半速轨迹不变、原节点完整执行、实际反馈、并发排斥与停止。实机接口验证使用隔离 Fake 模拟 DM 的 POS_VEL 与负角度夹爪，不试动硬件。

## 文件

- `dist/`：无需构建的本地静态网站。
- `hardware_session.py`：原 DM 驱动生命周期及串口、反馈状态检查。
- `server.py`：标准库 HTTP 服务与延迟加载的 ROS 2 桥接。
- `rviz_session.py`：原生 RViz 专用显示会话及清理。
- `setup_rviz.py`：本地网页显示依赖的一次性安装。
- `export_models.py`：从仓库 URDF 导出网页模型。
- `tests/`：输入校验、HTTP 安全边界及可选 ROS 集成测试。
- `dist/vendor/`：Three.js r170、OrbitControls、STLLoader 及 MIT 许可证。

模型来自原仓库，沿用其许可约定。Three.js 使用 MIT 许可证，见 `dist/vendor/THREE-LICENSE.txt`。

noVNC 使用 MPL 2.0，附带依赖和完整许可信息见 `dist/vendor/novnc/LICENSES.txt`。

## 网页视觉、语音与文字交互

进入“视觉与交互”即可点击开启或关闭手势动作、手掌位置跟随、本地 Whisper 语音、文字与 MCP。页面同时显示原生 RViz 与带手部骨架、识别标签的摄像头画面；无需在终端启动节点。摄像头和麦克风由浏览器请求权限，可在页面选择摄像头。图像和录音只发往配置的本机桥接，不使用云端识别。

手势动作复用原项目的 MediaPipe 分类与去抖，石头、布、比心分别对应原版出拳、八字和爱心。手掌跟随启动原来的 `palm_follow_node`，沿用原启动参数，自动进入起手位。两种模式互斥；跟随期间手动关节和其他动作不可抢占。输入或反馈中断会关闭本页启动的视觉节点并保持当前位置，不自动恢复。

语音通过浏览器录制最多 8 秒，在本机缓存的 Whisper small 模型中识别，沿用原项目中文动作解析及置信度过滤。文字输入使用同一解析器。动作倍率在开启功能时生效，适用于手势、语音、文字动作；跟随保持原参数。MCP 服务只绑定环回地址，运动工具锁定；网页文字动作通过已有桥接的运动校验执行。

`vision_worker.py` 只替换原视觉节点的摄像头输入与窗口输出，原 ROS 源码和命令行启动方式保持原样。`feature_session.py` 管理本页创建的进程、输入超时及控制占用。仅额外 MCP 依赖通过 `setup_features.py` 安装到项目的 `.feature-venv`，不修改系统或 Conda 包。部署完成后，日常操作全部通过网页。

隔离 Fake 集成验证：`tests/integration_features.py` 使用域 197，覆盖真实 MediaPipe 推理、手势对应原动作与倍率、跟随原起手位、输入中断、离线语音模型和 MCP 生命周期；不访问实机。

真机页面集成验证：`tests/integration_hardware.py` 使用域 198 / 199，只以原 Fake 驱动模拟 DM 接口，不打开真实串口；覆盖原驱动启动参数、全局目标切换、六轴反馈、单次单关节目标、电机速度上限、LOWLEVEL_STREAMING 连续操作、负角度夹爪、原版半速爱心、停止、断线及显式返回仿真。

## 原动作执行回报与误超时保护

网页启动的原舞蹈、问候和轨迹节点仍执行原类、原配置与原定时器。`action_worker.py` 只旁路记录其 `_publish_status` 和 `_tick` 的实际步骤游标，不重放或修改指令。原状态 Topic 使用独立回调组；即使 DDS 错过仅发布一次的开始或结束消息，网页仍能从自己拥有的本地子进程读取有序回报，不会因为缺少一条开始消息而在执行约 12 秒后误停止。

动作监测依据真实步骤是否继续推进，保留反馈新鲜度、控制器故障、安全锁止、步骤停滞与总执行时间限制。失败时保留原节点日志尾部及 `stop_origin` / `stop_reason`，区分网页监测停止与手动停止。只监测和清理本页创建的动作进程，不自动解锁或重新执行。

`tests/integration_action_feedback.py` 在隔离 Fake 域 201 故意丢弃所有 ROS 动作状态，完整执行原版舞蹈、半速问候和爱心，并验证到位、无误锁止及软件停止仍有效。

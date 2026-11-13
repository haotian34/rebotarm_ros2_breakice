# rebotarm_gesture_actions —— 任务三：手势触发的 TCP 轨迹动作

手势 → 关节轨迹回放。三条轨迹全部由 `precompute_gesture_tracks.py` 用官方
SDK 位置-only IK **离线预计算**（30Hz 采样，FK 误差 <2mm、限位余量 >0.05rad、
关节速度按DM SDK配置逐关节校验：关节1~3 <5rad/s、关节4~6 <3rad/s，校验通过才写文件），运行时不在线解 IK。

| 轨迹 | 手势 | 动作（均以「悬空鸡脖子」home 姿态为首尾） |
|---|---|---|
| `rock_push` | ✊ | TCP在前方30~50cm、高20cm处，20cm行程；约0.55s快出、1.36s慢收×3；原路归位（约8.5s） |
| `paper_figure8` | ✋ | Y-Z面26×14cm大8字×2圈，叠加手腕扭转（约3.6s） |
| `heart_draw` | 🫶 | 两笔画心，高22cm、宽约24.5cm，每半心约2.27s（全程约7.8s） |
| `paper_circle` | （旧版） | Y-Z面直径10cm圆×2圈（约6.3s），可手动播放 |

> 画圆/爱心的展示平面是 Y-Z（左右×上下），正对机械臂前方的观众看到完整图形。

出拳参数在生成器的 `ROCK_PUSH` 中：`center_x`/`height` 是基座坐标系中的中心和高度，
`amplitude` 是半行程。回收和归位反向复用前冲/蓄势关节轨迹，避免位置-only IK 的姿态漂移。
播放器 `vlim`、`rock_vlim`、`figure8_vlim` 默认均为5；关节1~3上限5rad/s，关节4~6上限3rad/s，沿用DM SDK配置。可显式设置更低值。
这些是软件设定，离线校验不代表已验证实机带负载的跟踪性能。

## 节点

- `trajectory_player_node`：通用轨迹播放器。
  订阅 `/rebotarm/action/play`（String，轨迹名）触发；先 1.5s 余弦缓动从
  当前姿态接入轨迹首帧，再 30Hz 逐帧回放（逐帧限位钳制）；
  回放结束后自动复位到全零初始位（`reset_pose` 参数：zeros/home/none，
  launch 参数 `reset_pose:=home` 可改为停在悬空鸡脖子），复位完成后
  发布 `/rebotarm/action/status`（`BUSY:<name>` / `IDLE`）；BUSY 期间忽略新请求。
- `gesture_dispatcher_node`：订阅 `/rebotarm/gesture_cmd`，映射
  rock→rock_push / paper→paper_figure8 / heart→heart_draw；
  状态机 READY→BUSY→COOLDOWN(2s)→READY，BUSY/冷却期间忽略手势。
  **边沿触发**：同一手势持续举着只执行一次；放下手（识别为 none）后
  再次举起才触发下一次。视觉端的心跳重发不会导致重复执行。

## 使用

```bash
# 修改动作参数后重新生成轨迹（改脚本顶部 ROCK_PUSH / PAPER_CIRCLE / HEART_DRAW）
python3 src/rebotarm_gesture_actions/rebotarm_gesture_actions/precompute_gesture_tracks.py
colcon build --packages-select rebotarm_gesture_actions

# 启动（需先启动 fake_dm_bringup 或真机驱动）
ros2 launch rebotarm_gesture_actions gesture_actions.launch.py

# 手动触发单个动作
ros2 topic pub --once /rebotarm/action/play std_msgs/msg/String "{data: rock_push}"
```

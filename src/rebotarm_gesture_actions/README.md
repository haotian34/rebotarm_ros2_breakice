# rebotarm_gesture_actions —— 任务三：手势触发的 TCP 轨迹动作

手势 → 关节轨迹回放。三条轨迹全部由 `precompute_gesture_tracks.py` 用官方
SDK 位置-only IK **离线预计算**（30Hz 采样，FK 误差 <2mm、限位余量 >0.05rad、
关节速度 <0.95rad/s 四项校验通过才写文件），运行时不在线解 IK。

| 轨迹 | 手势 | 动作（均以「悬空鸡脖子」home 姿态为首尾） |
|---|---|---|
| `rock_push` | ✊ | TCP 沿基座 X 轴前后 ±6cm，快出慢收 ×3（约 7s） |
| `paper_figure8` | ✋ | TCP 在 Y-Z 竖直面画 26×14cm 大 8 字 ×2 圈 + j6 手腕扭转，全身关节参与（约 8s） |
| `heart_draw` | 🫶 | **两笔画心**：右半心（顶部→底部）→ 抬笔回顶部 → 左半心（顶部→底部），高 22cm（约 8.6s） |
| `paper_circle` | （旧版） | TCP 在 Y-Z 竖直面画直径 10cm 圆 ×2 圈，保留在轨迹库中可手动播放 |

> 画圆/爱心的展示平面是 Y-Z（左右×上下），正对机械臂前方的观众看到完整图形。

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

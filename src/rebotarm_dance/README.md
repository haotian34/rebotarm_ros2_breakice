# rebotarm_dance —— 任务二：舞蹈表演

ROS2 Jazzy / reBot Arm B601-DM，仿真与真机通用（接口一致）。

## 运行

```bash
# 终端1（每个新终端都要先 source）
source /opt/ros/jazzy/setup.bash && source ~/rebotarm_ros2/install/setup.bash
ros2 launch rebotarm_bringup fake_dm_bringup.launch.py   # 虚拟驱动 + RViz

# 终端2
ros2 launch rebotarm_dance dance.launch.py               # 等待触发
# 或自动开演：ros2 launch rebotarm_dance dance.launch.py auto_start:=true

# 终端3 触发
ros2 topic pub --once /rebotarm/dance/start std_msgs/msg/Bool "{data: true}"
# 看状态：ros2 topic echo /rebotarm/dance/status
```

## 表演流程（config/dance.yaml）

开场抬臂 → 亮爪 → **机械街舞×2 圈**（TCP 钉在空中，身体律动）→ 复位
→ **空间滑步**（矩形 左上→左下→右下→右上→左上，夹爪大致朝前）
→ 复位 → 高举指天结束 pose（悬空定格 1s）→ 收爪回零。全程约 38s。

## 改动作

- 改顺序/时长/触发：只改 `config/dance.yaml`（move/track/gripper/wait/oscillate）。
- 改舞步本身：改 `rebotarm_dance/precompute_dance.py` 顶部 `GROOVE` / `SLIDE`
  参数，然后在**源码树**里重跑（依赖相对路径找 URDF 和官方 SDK）：

  ```bash
  python3 src/rebotarm_dance/rebotarm_dance/precompute_dance.py
  colcon build --packages-select rebotarm_dance
  ```

  脚本会重新离线解 IK 并对每帧做 FK 校验（TCP 误差 <2mm、插值中点漂移
  <2mm、限位余量 >0.05rad、关节速度 <0.95rad/s），不达标会拒绝写文件。

## 实现要点

- `dance_node.py`：与 rebotarm_greeting 同一套执行器框架（30Hz 余弦缓动、
  BUSY/IDLE 状态机、限位钳制），新增 `track` 步骤逐帧回放预计算轨迹，
  运行时**不在线解 IK**。
- `precompute_dance.py`：基于官方 SDK（Pinocchio）离线生成 30Hz 轨迹。
  街舞段用**加权 CLIK**（身体关节贵、腕部便宜，实测比官方等权 DLS 多保留
  约一倍摆幅）；滑步段用官方 `solve_ik` 位置 IK + 链式初值。
- 已实测（fake_dm_bringup 无头仿真 + joint_states 录数据 FK 复核）：
  街舞 TCP 偏移均值 0.9mm / 峰值 7.9mm（峰值来自驱动限速跟踪滞后）；
  滑步矩形 y=±0.220m 精确；全程零限位违规。

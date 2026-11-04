"""
reBot Arm 任务二：舞蹈表演节点

工作方式（与 rebotarm_greeting 同一套执行器框架）：
  1. 订阅 /{ns}/dance/start (std_msgs/Bool)，收到 true 开始表演
  2. 按 config/dance.yaml 中的剧本逐步执行，30Hz 平滑插值
     向 /{ns}/joints/jointN/cmd/pos_vel 发送位置指令
  3. 执行期间处于 BUSY，忽略新的触发；结束后回到 IDLE
  4. 该节点对仿真驱动和真实机械臂完全通用（接口一致）

与 greeting 的唯一区别是新增步骤类型 track：
  回放 config/dance_tracks.yaml 中由 precompute_dance.py 离线 IK
  生成的 30Hz 关节轨迹（逐帧限位钳制后下发），运行时不在线解 IK。
"""

import math

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from rebotarm_msgs.msg import JointPosVelCmd

JOINTS = [f"joint{i}" for i in range(1, 7)]


class DanceNode(Node):
    def __init__(self):
        super().__init__("rebotarm_dance")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("rate_hz", 60.0)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("track_speed", 1.1)
        self.declare_parameter("config_path", "")
        self.declare_parameter("tracks_path", "")

        ns = self.get_parameter("arm_namespace").value.strip("/")
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.track_speed = max(0.1, float(self.get_parameter("track_speed").value))
        auto_start = bool(self.get_parameter("auto_start").value)

        share = get_package_share_directory("rebotarm_dance")
        config_path = self.get_parameter("config_path").value
        if not config_path:
            config_path = share + "/config/dance.yaml"
        with open(config_path, "r", encoding="utf-8") as f:
            self.cfg = yaml.safe_load(f)["dance"]
        self.get_logger().info(f"已加载动作配置: {config_path}")

        tracks_path = self.get_parameter("tracks_path").value
        if not tracks_path:
            tracks_path = share + "/config/dance_tracks.yaml"
        with open(tracks_path, "r", encoding="utf-8") as f:
            tracks_yaml = yaml.safe_load(f)
        # 轨迹采样率（生成时固定 30Hz，与节点节拍一致，逐帧回放）
        track_rate = float(tracks_yaml["rate_hz"])
        self.tracks = {
            name: {"rate": track_rate, "frames": t["frames"]}
            for name, t in tracks_yaml["tracks"].items()
        }
        self.get_logger().info(
            f"已加载轨迹: {tracks_path} -> "
            + ", ".join(f"{n}({len(t['frames'])}帧)" for n, t in self.tracks.items()))

        # ---- 指令发布器：6 个关节 + 夹爪 ----
        self.joint_pubs = {
            j: self.create_publisher(
                JointPosVelCmd, f"/{ns}/joints/{j}/cmd/pos_vel", 10
            )
            for j in JOINTS
        }
        self.gripper_pub = self.create_publisher(
            JointPosVelCmd, f"/{ns}/gripper/cmd/pos_vel", 10
        )

        # ---- 状态反馈 / 触发 ----
        self.status_pub = self.create_publisher(String, f"/{ns}/dance/status", 10)
        self.create_subscription(
            Bool, f"/{ns}/dance/start", self._on_start, 10
        )
        # 驱动以 sensor_data QoS（best_effort）发布关节状态，订阅端必须匹配
        self.create_subscription(
            JointState, f"/{ns}/joint_states", self._on_joint_states,
            qos_profile_sensor_data,
        )

        # ---- 运行时状态 ----
        self.state = "IDLE"          # IDLE / RUNNING
        self.current = {j: 0.0 for j in JOINTS}  # 最近一次已知关节角
        self.steps = []              # 展开后的执行步骤
        self.step_index = 0
        self.step_t = 0.0            # 当前步骤已进行时间
        self.step_from = {}          # 当前步骤起始关节角

        self.dt = 1.0 / self.rate_hz
        self.create_timer(self.dt, self._tick)

        self._publish_status("就绪：等待触发 (ros2 topic pub --once "
                             f"/{ns}/dance/start std_msgs/msg/Bool \"{{data: true}}\")")
        if auto_start:
            self._start()

    # ---------------- 触发与状态 ----------------

    def _on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current:
                self.current[name] = float(pos)

    def _on_start(self, msg: Bool):
        if not msg.data:
            return
        if self.state == "RUNNING":
            self.get_logger().warn("正在表演中，忽略本次触发")
            return
        self._start()

    def _publish_status(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _start(self):
        self.steps = self._build_steps()
        self.step_index = 0
        self.step_t = 0.0
        self.step_from = dict(self.current)
        self.state = "RUNNING"
        self._publish_status("开始表演：舞蹈（机械街舞 + 空间滑步）")

    # ---------------- 剧本展开 ----------------

    def _build_steps(self):
        """把 YAML 剧本展开为统一的内部步骤列表。"""
        cfg = self.cfg
        zeros = {j: 0.0 for j in JOINTS}
        steps = []
        for raw in cfg["sequence"]:
            t = raw["type"]
            if t == "move":
                if raw.get("target") == "zeros":
                    target = dict(zeros)
                else:
                    target = dict(cfg[raw["target"]])
                steps.append({"type": "move", "target": target,
                              "duration": float(raw["duration"])})
            elif t == "track":
                name = raw["name"]
                if name not in self.tracks:
                    raise KeyError(f"轨迹 '{name}' 不在 dance_tracks.yaml 中，"
                                   f"可用: {list(self.tracks)}")
                steps.append({"type": "track", "name": name,
                              "times": int(raw.get("times", 1))})
            elif t == "oscillate":
                steps.append({"type": "oscillate", "joint": raw["joint"],
                              "amplitude": float(raw["amplitude"]),
                              "period": float(raw["period"]),
                              "times": int(raw.get("times", 1))})
            elif t == "gripper":
                steps.append({"type": "gripper", "action": raw["action"],
                              "times": int(raw.get("times", 1)),
                              "dwell": float(raw.get("dwell", 0.4))})
            elif t == "wait":
                steps.append({"type": "wait", "duration": float(raw["duration"])})
        return steps

    # ---------------- 主循环（30Hz） ----------------

    def _tick(self):
        if self.state != "RUNNING":
            return

        if self.step_index >= len(self.steps):
            self.state = "IDLE"
            self._publish_status("表演结束，回到就绪状态")
            return

        step = self.steps[self.step_index]
        self.step_t += self.dt
        finished = self._run_step(step)
        if finished:
            self.step_index += 1
            self.step_t = 0.0
            self.step_from = dict(self.current)

    def _run_step(self, step) -> bool:
        """执行一步，返回该步骤是否完成。"""
        t = step["type"]

        if t == "move":
            duration = step["duration"]
            progress = min(1.0, self.step_t / duration)
            # 余弦缓动：两头慢、中间快，动作更自然
            eased = 0.5 - 0.5 * math.cos(math.pi * progress)
            for j in JOINTS:
                start = self.step_from[j]
                target = self._clamp(j, step["target"][j])
                self._command(j, start + (target - start) * eased)
            return progress >= 1.0

        if t == "track":
            # 按倍率回放并在相邻帧间插值，避免加速后出现跳帧和关节抖动
            tr = self.tracks[step["name"]]
            frames = tr["frames"]
            frame_pos = self.step_t * tr["rate"] * self.track_speed
            total_frames = len(frames) * step["times"]
            if frame_pos >= total_frames:
                return True
            frame_index = int(frame_pos)
            alpha = frame_pos - frame_index
            q0 = frames[frame_index % len(frames)]
            q1 = frames[(frame_index + 1) % len(frames)]
            q = [
                float(v0) + (float(v1) - float(v0)) * alpha
                for v0, v1 in zip(q0, q1)
            ]
            for j, v in zip(JOINTS, q):
                self._command(j, self._clamp(j, float(v)))
            return False

        if t == "oscillate":
            period = step["period"]
            total = step["times"] * period
            if self.step_t >= total:
                self._command(step["joint"], self.step_from[step["joint"]])
                return True
            phase = 2.0 * math.pi * (self.step_t / period)
            base = self.step_from[step["joint"]]
            value = base + step["amplitude"] * math.sin(phase)
            self._command(step["joint"], self._clamp(step["joint"], value))
            return False

        if t == "gripper":
            g = self.cfg["gripper"]
            dwell = step["dwell"]
            if step["action"] == "toggle":
                total = step["times"] * 2.0 * dwell
                phase_index = int(self.step_t / dwell)
                open_now = (phase_index % 2 == 0)
                self._command_gripper(g["open_pos"] if open_now else g["close_pos"])
                return self.step_t >= total
            pos = g["open_pos"] if step["action"] == "open" else g["close_pos"]
            self._command_gripper(pos)
            return self.step_t >= dwell

        if t == "wait":
            return self.step_t >= step["duration"]

        return True

    # ---------------- 底层指令 ----------------

    def _clamp(self, joint: str, value: float) -> float:
        lo, hi = self.cfg["joint_limits"][joint]
        return max(lo, min(hi, value))

    def _command(self, joint: str, pos: float):
        msg = JointPosVelCmd()
        msg.stamp = self.get_clock().now().to_msg()
        msg.pos = float(pos)
        msg.vlim = 1.0  # 与预计算轨迹和仿真驱动限速保持一致
        self.joint_pubs[joint].publish(msg)
        self.current[joint] = float(pos)

    def _command_gripper(self, pos: float):
        msg = JointPosVelCmd()
        msg.stamp = self.get_clock().now().to_msg()
        msg.pos = float(pos)
        msg.vlim = float(self.cfg["gripper"]["speed"])
        self.gripper_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

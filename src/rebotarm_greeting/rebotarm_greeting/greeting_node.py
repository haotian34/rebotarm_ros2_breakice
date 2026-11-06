"""
reBot Arm 任务一：三位置点头问候节点

工作方式：
  1. 订阅 /{ns}/greeting/start (std_msgs/Bool)，收到 true 开始表演
  2. 按 config/greeting.yaml 中的剧本逐步执行，30Hz 平滑插值
     向 /{ns}/joints/jointN/cmd/pos_vel 发送位置指令
  3. 执行期间处于 BUSY，忽略新的触发；结束后回到 IDLE
  4. 该节点对仿真驱动和真实机械臂完全通用（接口一致）
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


class GreetingNode(Node):
    def __init__(self):
        super().__init__("rebotarm_greeting")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("config_path", "")

        ns = self.get_parameter("arm_namespace").value.strip("/")
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        auto_start = bool(self.get_parameter("auto_start").value)

        config_path = self.get_parameter("config_path").value
        if not config_path:
            config_path = (
                get_package_share_directory("rebotarm_greeting")
                + "/config/greeting.yaml"
            )
        with open(config_path, "r", encoding="utf-8") as f:
            self.cfg = yaml.safe_load(f)["greeting"]
        self.get_logger().info(f"已加载动作配置: {config_path}")

        # 真机速度控制：time_scale 统一放慢所有时长，joint_vlim 限制关节速度
        speed_cfg = self.cfg.get("speed", {})
        self.time_scale = float(speed_cfg.get("time_scale", 1.0))
        self.joint_vlim = float(speed_cfg.get("joint_vlim", 1.5))
        self.get_logger().info(
            f"速度配置: time_scale={self.time_scale}, joint_vlim={self.joint_vlim} rad/s"
        )

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
        self.status_pub = self.create_publisher(String, f"/{ns}/greeting/status", 10)
        self.create_subscription(
            Bool, f"/{ns}/greeting/start", self._on_start, 10
        )
        # 驱动以 sensor_data QoS（best_effort）发布，订阅端必须匹配
        self.create_subscription(
            JointState, f"/{ns}/joint_states", self._on_joint_states,
            qos_profile_sensor_data,
        )

        # ---- 运行时状态 ----
        self.state = "IDLE"          # IDLE / RUNNING
        self.current = {j: 0.0 for j in JOINTS}  # 最近一次反馈关节角
        # 最近一次下发的指令角：步骤衔接以此为准（避免反馈滞后造成回跳）
        self.commanded = {j: 0.0 for j in JOINTS}
        self.steps = []              # 展开后的执行步骤
        self.step_index = 0
        self.step_t = 0.0            # 当前步骤已进行时间
        self.step_from = {}          # 当前步骤起始关节角

        self.dt = 1.0 / self.rate_hz
        self.create_timer(self.dt, self._tick)

        self._publish_status("就绪：等待触发 (ros2 topic pub --once "
                             f"/{ns}/greeting/start std_msgs/msg/Bool \"{{data: true}}\")")
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
        self.commanded = dict(self.current)
        self.step_from = dict(self.commanded)
        self.state = "RUNNING"
        self._publish_status("开始表演：三位置点头问候")

    # ---------------- 剧本展开 ----------------

    def _build_steps(self):
        """把 YAML 剧本展开为统一的内部步骤列表。"""
        cfg = self.cfg
        home = dict(cfg["home_pose"])
        zeros = {j: 0.0 for j in JOINTS}
        steps = []
        for raw in cfg["sequence"]:
            t = raw["type"]
            if t == "move":
                if "to_position" in raw:
                    pos = next(
                        p for p in cfg["positions"] if p["name"] == raw["to_position"]
                    )
                    target = dict(self.current)
                    target.update(home)
                    target["joint1"] = pos["joint1"]
                elif raw.get("target") == "zeros":
                    target = dict(zeros)
                else:
                    target = dict(cfg[raw["target"]])
                steps.append({"type": "move", "target": target,
                              "duration": float(raw["duration"]) * self.time_scale})
            elif t == "nod":
                n = cfg["nod"]
                steps.append({"type": "oscillate", "joint": n["joint"],
                              "amplitude": n["amplitude"],
                              "period": n["period"] * self.time_scale,
                              "times": int(raw.get("times", n["times"])),
                              # 鸡啄米：点头时夹爪联动开合
                              "beak": bool(raw.get("beak", n.get("beak", False)))})
            elif t == "wave":
                w = cfg["wave"]
                steps.append({"type": "oscillate", "joint": w["joint"],
                              "amplitude": w["amplitude"],
                              "period": w["period"] * self.time_scale,
                              "times": int(raw.get("times", 1))})
            elif t == "gripper":
                steps.append({"type": "gripper", "action": raw["action"],
                              "times": int(raw.get("times", 1)),
                              "dwell": float(raw.get("dwell", 0.4)) * self.time_scale})
            elif t == "wait":
                steps.append({"type": "wait",
                              "duration": float(raw["duration"]) * self.time_scale})
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
            self.step_from = dict(self.commanded)

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

        if t == "oscillate":
            period = step["period"]
            total = step["times"] * period
            if self.step_t >= total:
                # 收尾：确保回到该关节的基准角；喙收拢
                self._command(step["joint"], self.step_from[step["joint"]])
                if step.get("beak"):
                    self._command_gripper(self.cfg["gripper"]["close_pos"])
                return True
            phase = 2.0 * math.pi * (self.step_t / period)
            base = self.step_from[step["joint"]]
            value = base + step["amplitude"] * math.sin(phase)
            self._command(step["joint"], self._clamp(step["joint"], value))
            if step.get("beak"):
                # 头低到最低点(sin=-1)时喙张最开，头抬起时喙合上
                g = self.cfg["gripper"]
                fraction = 0.5 - 0.5 * math.sin(phase)
                beak_pos = g["close_pos"] + (g["open_pos"] - g["close_pos"]) * fraction
                self._command_gripper(beak_pos)
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
        msg.vlim = self.joint_vlim  # 速度上限（rad/s），驱动按此限速跟踪
        self.joint_pubs[joint].publish(msg)
        self.current[joint] = float(pos)
        self.commanded[joint] = float(pos)

    def _command_gripper(self, pos: float):
        msg = JointPosVelCmd()
        msg.stamp = self.get_clock().now().to_msg()
        msg.pos = float(pos)
        msg.vlim = float(self.cfg["gripper"]["speed"])
        self.gripper_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GreetingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
reBot Arm 任务三：通用关节轨迹播放器

按名字加载 config/gesture_tracks.yaml 中离线预计算的 30Hz 关节轨迹并回放：
  1. 订阅 /{ns}/action/play (std_msgs/String)，data 为轨迹名
  2. 收到请求时若 IDLE：先从当前关节角余弦缓动平滑接入轨迹首帧
     （entry_duration 秒，不假设当前一定在 home），再逐帧回放（限位钳制）
  3. 回放结束后进入 RESET 阶段：余弦缓动回到复位姿态
     （reset_pose=zeros 全零初始位 / home 悬空鸡脖子 / none 保持轨迹末帧）
  4. 执行期间 BUSY，忽略新的播放请求
  5. 发布 /{ns}/action/status (std_msgs/String)："BUSY:<name>" / "IDLE"
     （复位完成后才发布 IDLE）

该节点对仿真驱动和真实机械臂完全通用（接口一致），
执行器框架复用 rebotarm_greeting / rebotarm_dance 的 30Hz 状态机思路。
"""

import math

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from rebotarm_msgs.msg import JointPosVelCmd

JOINTS = [f"joint{i}" for i in range(1, 7)]

# 各关节安全限位（与 DM URDF 一致），回放逐帧钳制
# 注意：DM 的 joint2/joint3 是「负向范围」，绝对不要给正数！
JOINT_LIMITS = {
    "joint1": (-2.8, 2.8),
    "joint2": (-3.14, 0.0),
    "joint3": (-3.14, 0.0),
    "joint4": (-1.87, 1.57),
    "joint5": (-1.57, 1.57),
    "joint6": (-3.14, 3.14),
}


class TrajectoryPlayerNode(Node):
    def __init__(self):
        super().__init__("rebotarm_trajectory_player")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("tracks_path", "")
        self.declare_parameter("entry_duration", 1.5)  # 当前姿态 -> 轨迹首帧
        self.declare_parameter("vlim", 1.0)            # 关节速度上限（驱动限速）
        # 动作结束后的复位姿态：zeros=全零初始位 / home=悬空鸡脖子 / none=不复位
        self.declare_parameter("reset_pose", "zeros")
        self.declare_parameter("reset_duration", 2.0)

        ns = self.get_parameter("arm_namespace").value.strip("/")
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.entry_duration = float(self.get_parameter("entry_duration").value)
        self.vlim = float(self.get_parameter("vlim").value)
        self.reset_pose = str(self.get_parameter("reset_pose").value)
        self.reset_duration = float(self.get_parameter("reset_duration").value)
        reset_targets = {
            "zeros": [0.0] * 6,
            "home": [0.0, -0.62, -0.85, 0.0, 0.0, 0.0],
        }
        if self.reset_pose not in ("zeros", "home", "none"):
            self.get_logger().warn(
                f"未知 reset_pose [{self.reset_pose}]，按 zeros 处理")
            self.reset_pose = "zeros"
        self.reset_target = reset_targets.get(self.reset_pose)

        tracks_path = self.get_parameter("tracks_path").value
        if not tracks_path:
            tracks_path = (get_package_share_directory("rebotarm_gesture_actions")
                           + "/config/gesture_tracks.yaml")
        with open(tracks_path, "r", encoding="utf-8") as f:
            tracks_yaml = yaml.safe_load(f)
        track_rate = float(tracks_yaml["rate_hz"])
        if abs(track_rate - self.rate_hz) > 1e-6:
            self.get_logger().warn(
                f"轨迹采样率 {track_rate}Hz 与节点节拍 {self.rate_hz}Hz 不一致")
        self.tracks = {
            name: t["frames"] for name, t in tracks_yaml["tracks"].items()
        }
        self.get_logger().info(
            f"已加载轨迹: {tracks_path} -> "
            + ", ".join(f"{n}({len(t)}帧)" for n, t in self.tracks.items()))

        # ---- 指令发布器：6 个关节 ----
        self.joint_pubs = {
            j: self.create_publisher(
                JointPosVelCmd, f"/{ns}/joints/{j}/cmd/pos_vel", 10)
            for j in JOINTS
        }

        # ---- 触发 / 状态反馈 ----
        self.status_pub = self.create_publisher(String, f"/{ns}/action/status", 10)
        self.create_subscription(
            String, f"/{ns}/action/play", self._on_play, 10)
        # 驱动以 sensor_data QoS（best_effort）发布关节状态，订阅端必须匹配
        self.create_subscription(
            JointState, f"/{ns}/joint_states", self._on_joint_states,
            qos_profile_sensor_data)

        # ---- 运行时状态 ----
        self.state = "IDLE"                    # IDLE / ENTRY / TRACK
        self.current = {j: 0.0 for j in JOINTS}
        self.track_name = ""
        self.frames = []
        self.frame_index = 0
        self.entry_from = {}
        self.entry_t = 0.0

        self.dt = 1.0 / self.rate_hz
        self.create_timer(self.dt, self._tick)
        self._publish_status("IDLE")
        self.get_logger().info(
            f"轨迹播放器就绪：ros2 topic pub --once /{ns}/action/play "
            "std_msgs/msg/String \"{data: rock_push}\"")

    # ---------------- 触发与状态 ----------------

    def _on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current:
                self.current[name] = float(pos)

    def _on_play(self, msg: String):
        name = msg.data.strip()
        if self.state != "IDLE":
            self.get_logger().warn(f"正在播放 [{self.track_name}]，忽略请求 [{name}]")
            return
        if name not in self.tracks:
            self.get_logger().warn(
                f"未知轨迹 [{name}]，可用: {', '.join(self.tracks)}")
            # 自身仍在 IDLE，重发一次状态以防调度端卡在 BUSY
            self._publish_status("IDLE")
            return
        self.track_name = name
        self.frames = self.tracks[name]
        self.frame_index = 0
        self.entry_from = dict(self.current)
        self.entry_t = 0.0
        self.state = "ENTRY"
        self._publish_status(f"BUSY:{name}")
        self.get_logger().info(f"开始播放 [{name}]（{len(self.frames)} 帧，"
                               f"{len(self.frames) / self.rate_hz:.1f}s）")

    def _publish_status(self, text: str):
        self.status_pub.publish(String(data=text))

    # ---------------- 主循环（30Hz） ----------------

    def _tick(self):
        if self.state == "IDLE":
            return

        if self.state == "ENTRY":
            # 当前姿态 -> 轨迹首帧，余弦缓动平滑接入
            self.entry_t += self.dt
            u = min(1.0, self.entry_t / self.entry_duration)
            eased = 0.5 - 0.5 * math.cos(math.pi * u)
            first = self.frames[0]
            for i, j in enumerate(JOINTS):
                start = self.entry_from[j]
                self._command(j, start + (first[i] - start) * eased)
            if u >= 1.0:
                self.state = "TRACK"
            return

        if self.state == "RESET":
            # 轨迹末帧 -> 复位姿态，余弦缓动
            self.entry_t += self.dt
            u = min(1.0, self.entry_t / self.reset_duration)
            eased = 0.5 - 0.5 * math.cos(math.pi * u)
            for i, j in enumerate(JOINTS):
                start = self.entry_from[j]
                self._command(j, start + (self.reset_target[i] - start) * eased)
            if u >= 1.0:
                self.state = "IDLE"
                self._publish_status("IDLE")
                self.get_logger().info(
                    f"[{self.track_name}] 播放完成，已复位到 {self.reset_pose}，回到 IDLE")
            return

        # TRACK：逐帧回放（30Hz 采样与节点节拍一致）
        frame = self.frames[self.frame_index]
        for i, j in enumerate(JOINTS):
            self._command(j, frame[i])
        self.frame_index += 1
        if self.frame_index >= len(self.frames):
            if self.reset_target is not None:
                self.state = "RESET"
                self.entry_from = dict(self.current)
                self.entry_t = 0.0
                self.get_logger().info(
                    f"[{self.track_name}] 回放结束，复位到 {self.reset_pose}")
            else:
                self.state = "IDLE"
                self._publish_status("IDLE")
                self.get_logger().info(f"[{self.track_name}] 播放完成，回到 IDLE")

    # ---------------- 底层指令 ----------------

    def _command(self, joint: str, pos: float):
        lo, hi = JOINT_LIMITS[joint]
        pos = max(lo, min(hi, pos))
        msg = JointPosVelCmd()
        msg.stamp = self.get_clock().now().to_msg()
        msg.pos = float(pos)
        msg.vlim = self.vlim
        self.joint_pubs[joint].publish(msg)
        self.current[joint] = float(pos)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:
            pass


if __name__ == "__main__":
    main()

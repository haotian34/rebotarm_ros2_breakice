"""
reBot Arm 任务三：手势 -> 动作调度器

订阅 /{ns}/gesture_cmd（std_msgs/String: rock/paper/heart/none），
按映射表向 /{ns}/action/play 发送对应轨迹名：
  rock  -> rock_push     ✊ 出拳
  paper -> paper_figure8 ✋ 大8字 + 手腕扭转
  heart -> heart_draw    🫶 画爱心

状态机（订阅 /{ns}/action/status 反馈）：
  READY --收到有效手势--> BUSY（发 play 请求）
  BUSY  --收到 status=IDLE--> COOLDOWN（cooldown_sec 秒，忽略一切手势）
  COOLDOWN --计时到--> READY

边沿触发：视觉端按「变化 + 1Hz 心跳」发布手势，本节点只在手势
「由别的状态变为某个有效手势」时触发一次；持续举着同一手势不会
重复触发，放下手（none）后再次举起才会触发下一次。
BUSY / COOLDOWN 期间忽略所有手势；'none' 永远只用于复位边沿。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GestureDispatcherNode(Node):
    def __init__(self):
        super().__init__("rebotarm_gesture_dispatcher")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("cooldown_sec", 2.0)
        # 手势 -> 轨迹名 映射（可用参数覆盖）
        self.declare_parameter("map_rock", "rock_push")
        self.declare_parameter("map_paper", "paper_figure8")
        self.declare_parameter("map_heart", "heart_draw")

        ns = self.get_parameter("arm_namespace").value.strip("/")
        self.cooldown = float(self.get_parameter("cooldown_sec").value)
        self.mapping = {
            "rock": self.get_parameter("map_rock").value,
            "paper": self.get_parameter("map_paper").value,
            "heart": self.get_parameter("map_heart").value,
        }

        self.play_pub = self.create_publisher(String, f"/{ns}/action/play", 10)
        self.create_subscription(
            String, f"/{ns}/gesture_cmd", self._on_gesture, 10)
        self.create_subscription(
            String, f"/{ns}/action/status", self._on_status, 10)

        self.state = "READY"           # READY / BUSY / COOLDOWN
        self.cooldown_timer = None
        self.last_gesture = "none"     # 上一次收到的手势（边沿检测用）

        self.get_logger().info(
            f"调度器就绪：{self.mapping}，动作后冷却 {self.cooldown}s")

    # ---------------- 输入 ----------------

    def _on_gesture(self, msg: String):
        gesture = msg.data.strip()
        if gesture == self.last_gesture:
            return                      # 心跳/重复帧，非新手势边沿
        self.last_gesture = gesture
        if gesture not in self.mapping:
            return                      # none：复位边沿，等待下一次举手
        if self.state != "READY":
            self.get_logger().info(
                f"[{self.state}] 期间收到新手势 [{gesture}]，已忽略"
                "（如需再次触发，请先放下手等识别为 none 后再举起）")
            return
        track = self.mapping[gesture]
        self.play_pub.publish(String(data=track))
        self.state = "BUSY"
        self.get_logger().info(f"手势 [{gesture}] -> 播放轨迹 [{track}]")

    def _on_status(self, msg: String):
        if msg.data.strip() == "IDLE" and self.state == "BUSY":
            self.state = "COOLDOWN"
            self.cooldown_timer = self.create_timer(
                self.cooldown, self._on_cooldown_done)
            self.get_logger().info(f"动作结束，冷却 {self.cooldown}s")

    def _on_cooldown_done(self):
        if self.cooldown_timer is not None:
            self.cooldown_timer.cancel()
            self.cooldown_timer = None
        self.state = "READY"
        self.get_logger().info("冷却结束，恢复 READY")


def main(args=None):
    rclpy.init(args=args)
    node = GestureDispatcherNode()
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

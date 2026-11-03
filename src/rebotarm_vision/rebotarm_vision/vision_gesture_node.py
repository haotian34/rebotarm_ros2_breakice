"""
reBot Arm 任务三：视觉手势识别节点

笔记本摄像头 -> MediaPipe Hands（21 关键点）-> 纯几何规则分类（不训练模型）
-> 发布 /{ns}/gesture_cmd (std_msgs/String: rock / paper / heart / none)

特性：
  - 分类规则：双手入镜只判 heart/none；单手入镜才判 rock/paper/none
    （拇指不参与石头/布）
  - 去抖：连续 debounce_frames 帧判定一致才切换输出
  - 发布策略：状态变化时立即发布 + heartbeat_hz 心跳重发当前状态，
    保证 ros2 topic echo 随时有输出、下游随时能拿到当前手势
  - OpenCV 调试窗口画手部骨架（show_debug=false 可关，便于无显示环境）

分类规则本体在 gesture_classifier.py（纯函数，与 ROS 解耦），
阈值调节先用 scripts/test_gesture.py 实测，再通过参数覆盖。
"""

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rebotarm_vision.gesture_classifier import DEFAULT_PARAMS, classify


class VisionGestureNode(Node):
    def __init__(self):
        super().__init__("rebotarm_vision_gesture")

        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("show_debug", True)
        self.declare_parameter("debounce_frames", 5)
        self.declare_parameter("heartbeat_hz", 1.0)   # 当前手势心跳重发频率
        # 阈值参数（None = 用分类器默认值）
        self.declare_parameter("rock_ratio_max", 0.0)
        self.declare_parameter("paper_ratio_min", 0.0)

        ns = self.get_parameter("arm_namespace").value.strip("/")
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.show_debug = bool(self.get_parameter("show_debug").value)
        self.debounce = int(self.get_parameter("debounce_frames").value)
        self.heartbeat_period = 1.0 / max(
            float(self.get_parameter("heartbeat_hz").value), 0.01)
        self.last_pub_time = 0.0

        self.params = dict(DEFAULT_PARAMS)
        for key in ("rock_ratio_max", "paper_ratio_min"):
            v = float(self.get_parameter(key).value)
            if v > 0.0:
                self.params[key] = v

        self.pub = self.create_publisher(String, f"/{ns}/gesture_cmd", 10)

        self.cap = cv2.VideoCapture(int(self.get_parameter("camera_index").value))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,
                     int(self.get_parameter("frame_width").value))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,
                     int(self.get_parameter("frame_height").value))
        if not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头，请检查 camera_index 参数")

        self.hands_det = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=2,
            min_detection_confidence=0.7, min_tracking_confidence=0.5)
        self.draw = mp.solutions.drawing_utils

        self.history = []
        self.stable = "none"

        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"手势识别节点已启动 -> /{ns}/gesture_cmd "
            f"(rock/paper/heart/none, 去抖 {self.debounce} 帧)")

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("读取摄像头帧失败", throttle_duration_sec=2.0)
            return
        frame = cv2.flip(frame, 1)  # 镜像
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands_det.process(rgb)

        h, w = frame.shape[:2]
        hand_lms = []
        if res.multi_hand_landmarks:
            for hl in res.multi_hand_landmarks:
                if self.show_debug:
                    self.draw.draw_landmarks(
                        frame, hl, mp.solutions.hands.HAND_CONNECTIONS)
                hand_lms.append([(lm.x * w, lm.y * h) for lm in hl.landmark])

        raw = classify(hand_lms, self.params)

        # 去抖：连续 N 帧一致才切换稳定输出
        self.history.append(raw)
        if len(self.history) > self.debounce:
            self.history.pop(0)
        if (len(self.history) == self.debounce
                and len(set(self.history)) == 1 and raw != self.stable):
            self.stable = raw
            self._publish()

        # 心跳：周期性重发当前稳定手势（下游调度器按边沿触发，不会重复执行）
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_pub_time >= self.heartbeat_period:
            self._publish()

        if self.show_debug:
            color = {"rock": (0, 0, 255), "paper": (0, 255, 0),
                     "heart": (255, 0, 255)}.get(self.stable, (128, 128, 128))
            cv2.putText(frame, f"raw: {raw}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(frame, f"stable: {self.stable}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)
            cv2.imshow("rebotarm gesture", frame)
            cv2.waitKey(1)

    def _publish(self):
        self.pub.publish(String(data=self.stable))
        self.last_pub_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f"手势 -> {self.stable}")

    def destroy_node(self):
        try:
            self.cap.release()
            if self.show_debug:
                cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionGestureNode()
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

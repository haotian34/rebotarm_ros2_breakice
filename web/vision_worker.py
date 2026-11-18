"""Headless browser-camera adapter for the unchanged project vision node.

Only the capture/display boundaries are replaced, inside this owned subprocess.
The original MediaPipe detection, classification, debounce and palm gate run as-is.
"""
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped

from rebotarm_vision.vision_gesture_node import VisionGestureNode


class BrowserCapture:
    def __init__(self, *args):
        self.lock = threading.Lock()
        self.frame = None
        self.updated = 0.

    def set(self, *args):
        return True

    def isOpened(self):
        return True

    def receive(self, message):
        if len(message.data) > 350000:
            return
        frame = cv2.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.shape[0] > 720 or frame.shape[1] > 1280:
            return
        with self.lock:
            self.frame, self.updated = frame, time.monotonic()

    def read(self):
        with self.lock:
            frame, self.frame = self.frame, None
        return frame is not None, frame

    def release(self):
        pass


def main():
    rclpy.init()
    capture = BrowserCapture()
    # Changes are confined to this process, never the installed vision package.
    cv2.VideoCapture = lambda *args: capture
    cv2.waitKey = lambda *args: -1
    node = None

    def display(name, frame):
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if ok:
            message = CompressedImage()
            message.header.stamp = node.get_clock().now().to_msg()
            message.format, message.data = 'jpeg', encoded.tobytes()
            output.publish(message)

    cv2.imshow = display
    try:
        node = VisionGestureNode()
        ns = node.get_parameter('arm_namespace').value.strip('/')
        node.declare_parameter('web_preview_only', False)
        if node.get_parameter('web_preview_only').value:
            node.palm_pub = node.create_publisher(PointStamped, f'/{ns}/web_vision/palm_preview', qos_profile_sensor_data)
        output = node.create_publisher(CompressedImage, f'/{ns}/web_vision/annotated', qos_profile_sensor_data)
        node.create_subscription(CompressedImage, f'/{ns}/web_vision/input', capture.receive, qos_profile_sensor_data)

        def expire_palm():
            if time.monotonic() - capture.updated > .5:
                message = PointStamped()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = 'camera_normalized'
                node.palm_pub.publish(message)

        node.create_timer(.1, expire_palm)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

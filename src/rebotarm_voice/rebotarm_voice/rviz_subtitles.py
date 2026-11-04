"""Publish red, bottom-centered subtitles for RViz's native 2D overlay plugin."""
from html import escape

import rclpy
from rclpy.node import Node
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import String


def status_label(status):
    labels = {'LISTENING': '正在录音，请说出口令…',
              'TRANSCRIBING': '录音结束，正在本地识别…',
              'DRY_RUN': '识别成功（仅预览，不执行动作）',
              'REQUESTED': '已发送动作请求',
              'READY': '模型已就绪，等待录音',
              'IGNORED': '未执行：静音、口令不匹配或当前不可执行',
              'ERROR': '发生错误，请查看语音节点日志'}
    return labels.get(status.split(':', 1)[0], '正在加载本地语音模型…')



def make_overlay(text, status, width=640, text_size=22.0):
    msg = OverlayText()
    msg.action = OverlayText.ADD
    msg.width, msg.height = width, 64
    msg.horizontal_alignment = OverlayText.CENTER
    msg.vertical_alignment = OverlayText.BOTTOM
    msg.horizontal_distance = 0
    msg.vertical_distance = 12
    msg.text_size = float(text_size)
    msg.line_width = 1
    msg.font = 'Noto Sans CJK SC'
    msg.fg_color.r, msg.fg_color.a = 1.0, 1.0
    msg.bg_color.a = 0.0
    # The plugin uses rich text; escape recognized words rather than interpreting HTML.
    content = text.strip() or status_label(status)
    if len(content) > 100:
        content = content[:99] + '…'
    msg.text = '<div align="center">' + escape(content).replace('\n', '<br/>') + '</div>'
    return msg


class RvizSubtitlesNode(Node):
    def __init__(self):
        super().__init__('rebotarm_voice_subtitles')
        self.declare_parameter('arm_namespace', 'rebotarm')
        self.declare_parameter('subtitle_width', 640)
        self.declare_parameter('subtitle_text_size', 22.0)
        ns = self.get_parameter('arm_namespace').value.strip('/')
        self.width = self.get_parameter('subtitle_width').value
        self.text_size = self.get_parameter('subtitle_text_size').value
        if self.width < 100 or self.text_size <= 0:
            raise ValueError('Subtitle width must be >= 100 and text size positive')
        self.text, self.status = '', ''
        self.publisher = self.create_publisher(OverlayText, f'/{ns}/voice/subtitles', 1)
        self.create_subscription(String, f'/{ns}/voice/transcript', self._text, 10)
        self.create_subscription(String, f'/{ns}/voice/status', self._status, 10)
        self.create_timer(1.0, self._publish)
        self._publish()

    def _text(self, msg):
        self.text = msg.data
        self._publish()

    def _status(self, msg):
        self.status = msg.data
        if self.status == 'LISTENING':
            self.text = ''
        self._publish()

    def _publish(self):
        self.publisher.publish(make_overlay(self.text, self.status, self.width, self.text_size))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RvizSubtitlesNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

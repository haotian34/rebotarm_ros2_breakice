"""Push-to-talk Whisper node; all ROS mutations occur on executor callbacks."""
from concurrent.futures import ThreadPoolExecutor
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .audio import LocalRecognizer
from .commands import parse_command


class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('rebotarm_voice')
        defaults = dict(arm_namespace='rebotarm', execute_motion=False,
                        wake_word='机械臂', record_seconds=4.0, sample_rate=16000,
                        audio_device='', min_rms=0.01, language='zh',
                        model_size="base", model_device="cpu", compute_type="int8",
                        model_cache="", cpu_threads=4, max_command_age=25.0,
                        cooldown_sec=2.0, status_timeout=3.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {k: self.get_parameter(k).value for k in defaults}
        for key in ('record_seconds', 'max_command_age',
                    'cooldown_sec', 'status_timeout'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not 0 < self.cfg['record_seconds'] <= 15:
            raise ValueError('record_seconds must be <= 15')
        if self.cfg['sample_rate'] != 16000:
            raise ValueError('Local Whisper requires sample_rate=16000')
        if self.cfg['cpu_threads'] < 1:
            raise ValueError('cpu_threads must be positive')
        if not 0 < self.cfg['min_rms'] < 1:
            raise ValueError('min_rms must be between 0 and 1')
        ns = self.cfg['arm_namespace'].strip('/')
        if not ns:
            raise ValueError('arm_namespace must not be empty')
        self.play = self.create_publisher(String, f'/{ns}/action/play', 10)
        self.transcript = self.create_publisher(String, f'/{ns}/voice/transcript', 10)
        self.status = self.create_publisher(String, f'/{ns}/voice/status', 10)
        self.create_subscription(String, f'/{ns}/action/status', self._on_status, 10)
        self.create_service(Trigger, f'/{ns}/voice/listen', self._listen)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.recognizer = LocalRecognizer(**{k: self.cfg[k] for k in (
            'model_size', 'model_device', 'compute_type', 'model_cache', 'cpu_threads')})
        self.loading = True
        self.model_ready = False
        self.future = self.pool.submit(self.recognizer.load)
        self.player_state = ''
        self.status_at = float('-inf')
        self.block_until = 0.0
        self.pending = False
        self.pending_at = 0.0
        self.create_timer(0.1, self._poll)
        self.get_logger().info(
            f'Loading local Whisper {self.cfg["model_size"]}; /{ns}/voice/listen; '
            f'execute_motion={self.cfg["execute_motion"]}')

    def _report(self, message):
        self.status.publish(String(data=message))
        self.get_logger().info(message)

    def _on_status(self, msg):
        self.player_state = msg.data.strip()
        self.status_at = time.monotonic()
        if self.player_state.startswith('BUSY:'):
            self.pending = False
        elif self.player_state == 'IDLE' and not self.pending:
            # Only transition from BUSY starts cooldown; heartbeat must not extend it.
            if getattr(self, '_was_busy', False):
                self.block_until = time.monotonic() + self.cfg['cooldown_sec']
        self._was_busy = self.player_state.startswith('BUSY:')

    def _ready(self):
        return (not self.pending and self.player_state == 'IDLE'
                and time.monotonic() - self.status_at < self.cfg['status_timeout']
                and time.monotonic() >= self.block_until
                and self.play.get_subscription_count() == 1)

    def _listen(self, request, response):
        if not self.model_ready:
            response.success, response.message = False, 'Local model not ready; check voice/status and logs'
            return response
        if self.future is not None:
            response.success, response.message = False, 'Recording/transcribing already in progress'
            return response
        if self.cfg['execute_motion'] and not self._ready():
            response.success, response.message = False, 'Player unavailable, busy, or cooling down'
            return response
        device = self.cfg['audio_device'] or None
        if device and device.isdigit():
            device = int(device)
        self.started_at = time.monotonic()
        self.future = self.pool.submit(
            self.recognizer.record_and_transcribe, self.cfg['record_seconds'], self.cfg['sample_rate'],
            device, self.cfg['min_rms'], self.cfg['language'])
        response.success, response.message = True, 'Recording started; see voice/status and voice/transcript'
        self._report('LISTENING')
        return response

    def _poll(self):
        if self.pending and time.monotonic() - self.pending_at > self.cfg['status_timeout']:
            # Never retry an unacknowledged physical command automatically.
            self.pending = False
            self.player_state = ''
            self.status_at = float('-inf')
            self.block_until = time.monotonic() + self.cfg['cooldown_sec']
            self._report('ERROR: player acknowledgement timeout; command not retried')
        if self.future is None or not self.future.done():
            return
        future, self.future = self.future, None
        if self.loading:
            self.loading = False
            try:
                future.result()
            except Exception as exc:
                self._report(f'ERROR: local model load failed ({type(exc).__name__}); '
                             'run ros2 run rebotarm_voice download_model, then restart; '
                             'also check model_device/compute_type')
                return
            self.model_ready = True
            self._report('READY: local Whisper loaded; call voice/listen to record')
            return
        try:
            text = future.result()
        except Exception as exc:
            self._report(f'ERROR: recording/transcription failed ({type(exc).__name__}); check local model and microphone')
            return
        self.transcript.publish(String(data=text))
        track = parse_command(text, self.cfg['wake_word'])
        if not track:
            self._report('IGNORED: silence or unsupported command')
        elif time.monotonic() - self.started_at > self.cfg['max_command_age']:
            self._report('IGNORED: expired command')
        elif not self.cfg['execute_motion']:
            self._report(f'DRY_RUN:{track}')
        elif not self._ready():
            self._report('IGNORED: player unavailable, busy, or cooling down')
        else:
            self.pending = True
            self.pending_at = time.monotonic()
            self.play.publish(String(data=track))
            self._report(f'REQUESTED:{track}')

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

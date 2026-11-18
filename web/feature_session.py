"""Local feature lifecycle; browser inputs, original ROS nodes, owned processes."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
FEATURES = {'camera', 'gesture', 'palm', 'voice', 'agent'}
VISION = {'camera', 'gesture', 'palm'}
TRACKS = {'rock_push': 'rock', 'paper_figure8': 'paper', 'heart_draw': 'heart', 'paper_circle': 'circle'}


class FeatureSession:
    def __init__(self, bridge):
        self.owner = bridge
        self.lock = threading.RLock()
        self.transition = threading.RLock()
        self.states = {name: {'status': 'stopped', 'error': ''} for name in FEATURES}
        self.processes = {}
        self.logs = {}
        self.bridge = None
        self.vision = None
        self.speed = 1.
        self.frame_at = self.result_at = 0.
        self.jpeg = None
        self.frame_sequence = 0
        self.last_gesture = 'none'
        self.action_sequence = None
        self.block_until = 0.
        self.follow_lease = False
        self.voice_busy = False
        self.recognizer = None
        self.voice_load_lock = threading.Lock()
        self.voice_generation = 0
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self._watch, name='web-features', daemon=True)
        self.thread.start()

    def state(self):
        with self.lock:
            return {'features': {name: dict(value) for name, value in self.states.items()},
                    'vision': self.vision, 'frame_sequence': self.frame_sequence,
                    'frame_age_sec': None if not self.result_at else time.monotonic()-self.result_at,
                    'speed': self.speed, 'control_reserved': self.follow_lease,
                    'voice_busy': self.voice_busy}

    def _bridge(self):
        bridge = getattr(self.owner, 'bridge', self.owner)
        if bridge is None or bridge.state().get('mode') != 'ros':
            raise ValueError('当前机械臂尚未就绪，请先在页面启动仿真或连接真机。')
        if bridge is not self.bridge:
            from sensor_msgs.msg import CompressedImage
            from rclpy.qos import qos_profile_sensor_data
            self.bridge = bridge
            self.input_pub = bridge.node.create_publisher(CompressedImage, f'/{bridge.namespace}/web_vision/input', qos_profile_sensor_data)
            self.transcript_pub = bridge.node.create_publisher(bridge.String, f'/{bridge.namespace}/voice/transcript', 10)
            self.output_sub = bridge.node.create_subscription(CompressedImage, f'/{bridge.namespace}/web_vision/annotated', self._image, qos_profile_sensor_data)
            self.Image = CompressedImage
        return bridge

    def _image(self, message):
        with self.lock:
            if not self.vision:
                return
            self.jpeg = bytes(message.data)
            self.result_at = time.monotonic()
            self.frame_sequence += 1
            if self.states[self.vision]['status'] == 'starting':
                self.states[self.vision]['camera_ready'] = True
                if self.vision != 'palm':
                    self.states[self.vision]['status'] = 'running'

    def frame(self, data):
        if not data.startswith(b'\xff\xd8') or len(data) > 350000:
            raise ValueError('摄像头画面必须是小于 350KB 的 JPEG。')
        with self.lock:
            if not self.vision:
                raise ValueError('请先选择手势识别或手掌跟随。')
            bridge = self._bridge()
            self.frame_at = time.monotonic()
            message = self.Image()
            message.header.stamp = bridge.node.get_clock().now().to_msg()
            message.format, message.data = 'jpeg', data
            self.input_pub.publish(message)

    def image(self):
        with self.lock:
            return self.frame_sequence, self.jpeg

    def _spawn(self, key, command):
        bridge = self._bridge()
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(bridge.domain_id)
        env['MPLCONFIGDIR'] = '/tmp/rebotarm-web-matplotlib'
        env['PYTHONUNBUFFERED'] = '1'
        logfile = tempfile.TemporaryFile(mode='w+')
        process = subprocess.Popen(command, env=env, stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
        with self.lock:
            self.processes[key], self.logs[key] = process, logfile

    def _terminate(self, key):
        with self.lock:
            process = self.processes.pop(key, None)
            logfile = self.logs.pop(key, None)
        if process and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
            except ProcessLookupError:
                pass
        if logfile:
            logfile.close()

    def _hold(self):
        bridge = self.bridge
        if not bridge:
            return
        try:
            bridge._check('move_joints')
            with bridge.lock:
                positions = [float(bridge.positions[name]) for name in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')]
                opening = bridge.gripper_opening
                fake = bridge.status.get('mode') == 'fake_rs_pos_vel'
            for publisher, position in zip(bridge.fake_publishers, positions):
                message = bridge.JointPosVelCmd()
                message.pos, message.vlim = position, .5
                message.stamp = bridge.node.get_clock().now().to_msg()
                publisher.publish(message)
            if opening is not None:
                message = bridge.JointPosVelCmd()
                message.pos, message.vlim = float(opening)/18*(1 if fake else -1), .5
                message.stamp = bridge.node.get_clock().now().to_msg()
                bridge.web_gripper_hold.publish(message)
        except Exception:
            pass  # Never override a safety latch or stale/disabled controller.

    def _stop_vision(self):
        with self.lock:
            kind, self.vision = self.vision, None
            self.jpeg = None
            self.frame_sequence = 0
            self.result_at = 0.
            self.last_gesture = 'none'
            sequence = self.action_sequence
            self.action_sequence = None
            self.block_until = 0.
        if not kind:
            return
        self._terminate('palm')
        self._terminate('vision')
        bridge = self.bridge
        if bridge:
            with bridge.lock:
                own_action = sequence is not None and bridge.action_motion.get('sequence') == sequence and bridge.action_thread
            if own_action:
                bridge.cancel_manual()
                worker = bridge.action_thread
                if worker:
                    worker.join(timeout=5)
            if self.follow_lease or own_action:
                self._hold()
            if self.follow_lease:
                bridge.motion_lock.release()
        with self.lock:
            self.follow_lease = False
            self.states[kind] = {'status': 'stopped', 'error': ''}

    def start(self, feature, speed=1.):
        if feature not in FEATURES or isinstance(speed, bool) or not isinstance(speed, (int,float)) or not .25 <= speed <= 1:
            raise ValueError('请选择有效功能和 0.25–1.0 动作倍率。')
        with self.transition:
            bridge = self._bridge()
            with self.lock:
                if self.states[feature]['status'] in ('starting', 'running'):
                    return self.state()
            if feature in ('gesture','palm','voice','agent'):
                bridge._check('run_action')
            if feature in VISION:
                previous = self.vision
                self._stop_vision()
                if feature in ('gesture', 'palm'):
                    until = time.monotonic()+1
                    while True:
                        try:
                            bridge._ensure_original_available()
                            break
                        except Exception:
                            if not previous or time.monotonic() >= until:
                                raise
                            time.sleep(.02)  # Let DDS remove the owned node's endpoints.
                if feature == 'palm':
                    if not bridge.motion_lock.acquire(blocking=False):
                        raise ValueError('机械臂正在运动，请等待完成再开启跟随。')
                    self.follow_lease = True
                with self.lock:
                    self.vision, self.speed = feature, float(speed)
                    self.frame_at = self.result_at = 0.
                    self.states[feature] = {'status': 'starting', 'error': '', 'started_at': time.monotonic()}
                try:
                    command = ['/usr/bin/python3', str(ROOT/'web/vision_worker.py'), '--ros-args', '-p', f'arm_namespace:={bridge.namespace}', '-p', 'show_debug:=true', '-p', 'camera_mode:=manual', '-p', 'publish_gestures:='+('true' if feature == 'gesture' else 'false')]
                    command += ['-p','web_preview_only:='+('true' if feature == 'camera' else 'false')]
                    if feature == 'palm':
                        command += ['-p','model_complexity:=0','-p','palm_grace_sec:=0.10','-p','rate_hz:=60.0']
                    self._spawn('vision', command)
                except Exception:
                    self._stop_vision()
                    raise
            elif feature == 'voice':
                if self.follow_lease:
                    raise ValueError('请先关闭手掌跟随，再使用语音动作。')
                with self.lock:
                    self.speed = float(speed)
                    self.states['voice'] = {'status': 'starting', 'error': ''}
                    self.voice_generation += 1
                    generation = self.voice_generation
                threading.Thread(target=self._load_voice, args=(generation,), daemon=True).start()
            else:
                self.speed = float(speed)
                interpreter = ROOT/'web/.feature-venv/bin/python'
                with socket.socket() as listener:
                    listener.bind(('127.0.0.1',0))
                    port = listener.getsockname()[1]
                self._spawn('agent', [str(interpreter), '-m', 'rebotarm_agent.rebotarm_mcp_server', '--namespace', bridge.namespace, '--host', '127.0.0.1', '--port', str(port), '--motion-mode', 'locked'])
                self.states['agent'] = {'status': 'starting', 'error': '', 'port': port, 'started_at': time.monotonic()}
            return self.state()

    def stop(self, feature):
        if feature not in FEATURES:
            raise ValueError('未知功能。')
        with self.transition:
            if feature in VISION:
                self._stop_vision()
            elif feature == 'voice':
                self.voice_generation += 1
                self.states['voice'] = {'status':'stopped','error':''}
            else:
                self._terminate('agent')
                self.states['agent'] = {'status':'stopped','error':''}
            return self.state()

    def _load_voice(self, generation):
        try:
            from rebotarm_voice.audio import LocalRecognizer
            with self.voice_load_lock:
                recognizer = self.recognizer or LocalRecognizer()
                if recognizer.model is None:
                    recognizer.load()
                self.recognizer = recognizer
            with self.lock:
                if generation == self.voice_generation and self.states['voice']['status'] == 'starting':
                    self.states['voice'] = {'status':'running','error':''}
        except Exception as exc:
            with self.lock:
                if generation == self.voice_generation and self.states['voice']['status'] == 'starting':
                    self.states['voice'] = {'status':'error','error':f'本地语音模型加载失败：{exc}'}

    def text(self, text):
        from rebotarm_voice.commands import parse_command
        if not isinstance(text,str) or not 1 <= len(text) <= 200:
            raise ValueError('请输入 1–200 字的动作指令。')
        track = parse_command(text, '', True)
        if track not in TRACKS:
            raise ValueError('未识别为动作指令，请说“出拳”“画八字”“画爱心”或“画圆”。')
        bridge = self._bridge()
        result = bridge.command({'command':'run_action','action':TRACKS[track],'speed':self.speed})
        return {'text':text,'action':TRACKS[track],'accepted':result['accepted']}

    def audio(self, data):
        with self.lock:
            if self.states['voice']['status'] != 'running' or self.voice_busy:
                raise ValueError('语音模型尚未就绪或正在识别。')
            if not 3200 <= len(data) <= 320000 or len(data)%2:
                raise ValueError('录音需为 0.1–10 秒的 16kHz PCM。')
            self.voice_busy = True
            generation = self.voice_generation
        started = time.monotonic()
        try:
            import numpy as np
            waveform = np.frombuffer(data,dtype='<i2').astype(np.float32)/32768.
            if float(np.sqrt(np.mean(waveform**2))) < .01:
                raise ValueError('录音声音太小，请靠近麦克风重试。')
            segments,_ = self.recognizer.model.transcribe(waveform,language='zh',beam_size=5,temperature=0,condition_on_previous_text=False,vad_filter=True)
            segments = list(segments)
            text = ''.join(segment.text for segment in segments).strip()
            bridge = self._bridge()
            self.transcript_pub.publish(bridge.String(data=text))
            if not text or any(s.avg_logprob < -.8 or s.no_speech_prob > .6 for s in segments):
                raise ValueError('未获得可靠的语音指令，请重试。')
            with self.lock:
                if generation != self.voice_generation or self.states['voice']['status'] != 'running' or time.monotonic()-started > 25:
                    raise ValueError('语音已关闭或指令过期，不执行动作。')
            with self.transition:
                if generation != self.voice_generation or self.states['voice']['status'] != 'running':
                    raise ValueError('语音已关闭，不执行动作。')
                return self.text(text)
        finally:
            with self.lock:
                self.voice_busy = False

    def _watch(self):
        while not self.closed.wait(.1):
            try:
                with self.transition:
                    self._poll()
            except Exception as exc:
                with self.lock:
                    if self.vision:
                        self.states[self.vision]['error'] = str(exc)

    def _poll(self):
        now = time.monotonic()
        with self.lock:
            processes = list(self.processes.items())
        for key,process in processes:
            if process.poll() is not None:
                log = self.logs.get(key)
                if log:
                    log.seek(0)
                    error = log.read()[-1600:]
                else:
                    error = '功能进程已退出。'
                if key in ('vision','palm'):
                    with self.transition:
                        kind = self.vision
                        self._stop_vision()
                        if kind:
                            self.states[kind] = {'status':'error','error':error}
                else:
                    self._terminate(key)
                    self.states[key] = {'status':'error','error':error}
                return
        bridge = self.bridge
        if not bridge:
            return
        state = bridge.state()
        if self.vision:
            kind = self.vision
            age = now-self.frame_at if self.frame_at else now-self.states[kind].get('started_at',now)
            expired = age > (1.5 if self.frame_at else 20)
            lost_result = self.result_at and now-self.result_at > 2
            lost_result = lost_result or (not self.result_at and now-self.states[kind].get('started_at',now) > 25)
            unsafe = kind in ('gesture','palm') and (not state['motion_allowed'] or not state['enabled'] or not state['control_loop_active'] or state['error_codes'] or state['joint_age_sec'] is None or state['joint_age_sec'] > 1.5 or state['status_age_sec'] is None or state['status_age_sec'] > 2 or state['state_machine'].startswith('SAFETY_') or state['state_machine'] in ('FAULT','ERROR','GRAVITY_COMP'))
            if expired or lost_result or unsafe:
                with self.transition:
                    self._stop_vision()
                    self.states[kind] = {'status':'error','error':'摄像头输入、识别反馈或机械臂控制状态中断，功能已停止。请重新开启。'}
                return
            if kind == 'palm' and self.states[kind].get('camera_ready') and 'palm' not in self.processes:
                self._spawn('palm',['/usr/bin/python3','-m','rebotarm_vision.palm_follow_node','--ros-args','-p',f'arm_namespace:={bridge.namespace}'])
                self.states[kind]['follower_started_at'] = now
            if kind == 'palm' and 'palm' in self.processes and self.states[kind]['status'] == 'starting':
                output = os.pread(self.logs['palm'].fileno(),65536,0).decode(errors='replace')
                if 'DM 手掌跟随就绪' in output:
                    self.states[kind]['status'] = 'running'
                elif now-self.states[kind].get('follower_started_at',now) > 20:
                    self._stop_vision()
                    self.states[kind] = {'status':'error','error':'手掌跟随节点未能就绪，请重新开启。'}
                    return
            if kind == 'gesture' and self.result_at and now-self.result_at < .75:
                gesture = state.get('gesture','none')
                if gesture != self.last_gesture:
                    self.last_gesture = gesture
                    if gesture in ('rock','paper','heart') and now >= self.block_until and not bridge.action_thread:
                        result = bridge.command({'command':'run_action','action':gesture,'speed':self.speed})
                        self.action_sequence = result['sequence']
                        self.block_until = float('inf')
                if self.action_sequence is not None and not bridge.action_thread and self.block_until == float('inf'):
                    self.block_until = now+2.
        if self.states['agent']['status'] == 'starting':
            try:
                with socket.create_connection(('127.0.0.1',self.states['agent']['port']),timeout=.05):
                    self.states['agent']['status'] = 'running'
            except OSError:
                if now-self.states['agent']['started_at'] > 15:
                    self._terminate('agent')
                    self.states['agent'] = {'status':'error','error':'本地 MCP 服务未能就绪。'}

    def close(self):
        self.closed.set()
        self.thread.join(timeout=3)
        with self.transition:
            self._stop_vision()
            self._terminate('agent')
            self.voice_generation += 1
            self.states['voice'] = {'status':'stopped','error':''}

    def stop_all(self):
        with self.transition:
            self._stop_vision()
            self._terminate('agent')
            self.states['agent'] = {'status':'stopped','error':''}
            self.voice_generation += 1
            self.states['voice'] = {'status':'stopped','error':''}

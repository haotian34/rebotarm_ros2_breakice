#!/usr/bin/env python3
"""Dependency-free web server; optional ROS 2 bridge using installed ROS packages.

The bridge binds to loopback, is read-only unless --allow-motion is explicit,
checks browser origins, validates targets against URDF limits, and never enables
or starts the physical controller automatically.
"""
from __future__ import annotations

import argparse
from functools import partial
import importlib.util
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import socket
import signal
import subprocess
import tempfile
import threading
import time
from urllib.parse import urlsplit

DIST = Path(__file__).resolve().parent / 'dist'
JOINTS = tuple(f'joint{i}' for i in range(1, 7))
_rviz_spec = importlib.util.spec_from_file_location('rebotarm_rviz_session', Path(__file__).with_name('rviz_session.py'))
_rviz_module = importlib.util.module_from_spec(_rviz_spec)
_rviz_spec.loader.exec_module(_rviz_module)
RvizSession = _rviz_module.RvizSession
_action_spec = importlib.util.spec_from_file_location('rebotarm_web_actions', Path(__file__).with_name('action_program.py'))
_action_module = importlib.util.module_from_spec(_action_spec)
_action_spec.loader.exec_module(_action_module)
_worker_spec = importlib.util.spec_from_file_location('rebotarm_action_worker', Path(__file__).with_name('action_worker.py'))
_worker_module = importlib.util.module_from_spec(_worker_spec)
_worker_spec.loader.exec_module(_worker_module)
_hardware_spec = importlib.util.spec_from_file_location('rebotarm_hardware_session', Path(__file__).with_name('hardware_session.py'))
_hardware_module = importlib.util.module_from_spec(_hardware_spec)
_hardware_spec.loader.exec_module(_hardware_module)
_feature_spec = importlib.util.spec_from_file_location('rebotarm_web_features', Path(__file__).with_name('feature_session.py'))
_feature_module = importlib.util.module_from_spec(_feature_spec)
_feature_spec.loader.exec_module(_feature_module)


class BridgeError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise BridgeError(f'{name} must be a finite number')
    if not low <= value <= high:
        raise BridgeError(f'{name} must be within [{low}, {high}]')
    return float(value)


def require_dm(model):
    if model != 'dm':
        raise BridgeError('当前工作台仅支持 B601 DM。')


def validate_command(payload, model):
    require_dm(model)
    if not isinstance(payload, dict):
        raise BridgeError('Expected a JSON object')
    command = payload.get('command')
    if not isinstance(command, str) or command not in {'enable', 'disable', 'safe_home', 'safety_stop', 'move_joints', 'move_joint', 'gripper', 'run_action', 'stop_action'}:
        raise BridgeError('Unsupported command')
    result = {'command': command}
    if command == 'run_action':
        if set(payload)-{'command', 'action', 'speed'}:
            raise BridgeError('动作接口仅接受动作名和速度，不接受自定义指令。')
        catalog = json.loads((DIST / 'actions.json').read_text())['actions']
        action = payload.get('action')
        if not isinstance(action, str) or action not in catalog:
            raise BridgeError('未知的预设动作。')
        result.update(action=action, speed=number(payload.get('speed', 1.), 'speed', .25, 1.))
    if command == 'stop_action' and set(payload) != {'command'}:
        raise BridgeError('停止动作不接受额外参数。')
    if command == 'move_joints':
        positions = payload.get('positions')
        if not isinstance(positions, list) or len(positions) != 6:
            raise BridgeError('positions must contain exactly six joint angles in radians')
        manifest = json.loads((DIST / 'assets/models' / f'{model}.json').read_text())
        limits = {j['name']: (j['lower'], j['upper']) for j in manifest['joints']}
        result['positions'] = [number(v, name, *limits[name]) for name, v in zip(JOINTS, positions)]
        result['duration'] = number(payload.get('duration', 4), 'duration', 2, 30)
    if command == 'move_joint':
        joint = payload.get('joint')
        if joint not in JOINTS:
            raise BridgeError('请选择有效关节 joint1–joint6。')
        manifest = json.loads((DIST / 'assets/models' / f'{model}.json').read_text())
        limit = next(j for j in manifest['joints'] if j['name'] == joint)
        result['joint'] = joint
        result['position'] = number(payload.get('position'), 'position', limit['lower'], limit['upper'])
        result['speed'] = number(payload.get('speed', .3), 'speed', .05, .5)
    if command == 'gripper':
        result['opening_mm'] = number(payload.get('opening_mm'), 'opening_mm', 0, 90)
    return result


def auto_duration(delta, speed):
    """A zero-velocity cubic segment peaks at 1.5 * delta / duration.

    Use 1.6 for headroom, and allow arbitrarily long valid manual moves.
    """
    return max(.25, 1.6 * abs(delta) / speed)


def prepare_action(action, positions, speed, opening=0):
    """Preflight every original frame before any ROS command can be sent."""
    preset = json.loads((DIST / 'actions.json').read_text())['actions'][action]
    program = _action_module.render_program(action, positions, speed, opening)
    manifest = json.loads((DIST / 'assets/models/dm.json').read_text())
    limits = {joint['name']: (joint['lower'], joint['upper']) for joint in manifest['joints']}
    for frame in program['frames']:
        for name, value in zip(JOINTS, frame['joints']):
            number(value, name, *limits[name])
        number(frame['gripper'], 'gripper', 0, 90)
        number(frame['duration'], 'frame duration', .001, 1)
        for value, cap in zip(frame['vlim'], [5, 5, 5, 3, 3, 3]):
            number(value, 'original joint speed limit', 0, cap)
    return preset['title'], [{'kind': 'original', **program}]


class DemoBridge:
    def __init__(self, model='dm', namespace='rebotarm'):
        require_dm(model)
        self.model, self.namespace = model, namespace

    def state(self):
        return {'mode': 'demo', 'model': self.model, 'namespace': self.namespace,
                'motion_allowed': False, 'positions': {}, 'enabled': False,
                'joint_age_sec': None, 'status_age_sec': None, 'state_machine': 'DEMO'}

    def command(self, payload):
        validate_command(payload, self.model)
        raise BridgeError('ROS 2 is not connected. Restart with --ros.', 409)

    def close(self):
        pass


class RosBridge:
    def __init__(self, model='dm', namespace='rebotarm', allow_motion=False, *, domain_id=None):
        require_dm(model)
        # Lazy imports let the static website run without ROS or additional pip dependencies.
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.context import Context
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from control_msgs.action import FollowJointTrajectory
        from rebotarm_msgs.msg import ArmStatus, JointMotorState, JointPosVelCmd
        from rebotarm_msgs.srv import SetGripper
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import JointTrajectoryPoint

        self.rclpy = rclpy
        self.context = Context() if domain_id is not None else None
        rclpy.init(args=[], context=self.context, domain_id=domain_id,
                    signal_handler_options=SignalHandlerOptions.NO if self.context else None)
        self.model, self.namespace, self.allow_motion = model, namespace, allow_motion
        self.domain_id = domain_id if domain_id is not None else int(os.environ.get('ROS_DOMAIN_ID', '0'))
        self.original_status = {}
        self.lock = threading.RLock()
        self.motion_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.manual_pending = {}
        self.manual_thread = None
        self.manual_generation = 0
        self.manual_sequence = 0
        self.manual_motion = {'status': 'idle', 'sequence': 0}
        self.action_thread = None
        self.action_sequence = 0
        self.action_motion = {'status': 'idle', 'sequence': 0, 'progress': 0}
        self.active_goal = None
        self.joint_at = self.status_at = self.gripper_at = 0.0
        self.positions = {}
        self.velocities = {}
        self.gripper_opening = None
        self.status = {'enabled': False, 'state_machine': 'WAITING', 'error_codes': [], 'control_loop_active': False}
        self.gesture = self.transcript = ''
        self.motor_states = {}
        self.node = Node('rebotarm_web_bridge', context=self.context)
        self.group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        prefix = f'/{namespace}'
        self.subscriptions = [
            self.node.create_subscription(JointState, f'{prefix}/joint_states', self._joints, qos_profile_sensor_data),
            self.node.create_subscription(ArmStatus, f'{prefix}/arm_status', self._status, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)),
            self.node.create_subscription(JointMotorState, f'{prefix}/gripper/state', self._gripper, qos_profile_sensor_data),
            self.node.create_subscription(String, f'{prefix}/gesture_cmd', lambda msg: self._text('gesture', msg), 10),
            self.node.create_subscription(String, f'{prefix}/voice/transcript', lambda msg: self._text('transcript', msg), 10),
        ]
        for name in JOINTS:
            self.subscriptions.append(self.node.create_subscription(JointMotorState,f'{prefix}/joints/{name}/state',lambda msg,name=name:self._motor_state(name,msg),qos_profile_sensor_data))
        for kind in ('greeting', 'dance', 'action'):
            self.subscriptions.append(self.node.create_subscription(String, f'{prefix}/{kind}/status', lambda msg, kind=kind: self._original_status(kind, msg), 10, callback_group=self.group))
        self.original_play_pub = self.node.create_publisher(String, f'{prefix}/action/play', 10)
        self.original_start_pubs = {kind: self.node.create_publisher(Bool, f'{prefix}/{kind}/start', 10) for kind in ('greeting', 'dance')}
        self.Bool = Bool
        self.String = String
        self.triggers = {command: self.node.create_client(Trigger, f'{prefix}/{command}', callback_group=self.group) for command in ('enable', 'disable', 'safe_home', 'safety_stop')}
        self.gripper_client = self.node.create_client(SetGripper, f'{prefix}/gripper/set', callback_group=self.group)
        self.trajectory = ActionClient(self.node, FollowJointTrajectory, f'{prefix}/follow_joint_trajectory', callback_group=self.group)
        self.fake_publishers = [self.node.create_publisher(JointPosVelCmd, f'{prefix}/joints/{name}/cmd/pos_vel', 10) for name in JOINTS]
        self.web_gripper_hold = self.node.create_publisher(JointPosVelCmd, f'{prefix}/gripper/cmd/pos_vel', 10)
        self.JointPosVelCmd = JointPosVelCmd
        self.Trigger, self.SetGripper, self.FollowJointTrajectory, self.JointTrajectoryPoint = Trigger, SetGripper, FollowJointTrajectory, JointTrajectoryPoint

    def _joints(self, msg):
        positions = {name: float(value) for name, value in zip(msg.name, msg.position)}
        if not all(name in positions and math.isfinite(positions[name]) for name in JOINTS):
            return
        with self.lock:
            self.positions, self.joint_at = positions, time.monotonic()
            self.velocities = {name: float(value) for name, value in zip(msg.name, msg.velocity)
                               if math.isfinite(value)}

    def _motor_state(self,name,msg):
        if not all(math.isfinite(value) for value in (msg.position,msg.velocity,msg.torque)):
            return
        with self.lock:
            self.motor_states[name]={'position':float(msg.position),'velocity':float(msg.velocity),'torque':float(msg.torque),'status_code':int(msg.status_code),'received_at':time.monotonic()}

    def _original_status(self, kind, msg):
        with self.lock:
            self.original_status[kind] = (msg.data, time.monotonic())

    def _status(self, msg):
        with self.lock:
            self.status = {'enabled': bool(msg.enabled), 'state_machine': msg.state_machine,
                           'mode': msg.mode, 'control_loop_active': bool(msg.control_loop_active),
                           'error_codes': list(msg.error_codes)}
            self.status_at = time.monotonic()

    def _gripper(self, msg):
        value = float(msg.position)
        if math.isfinite(value):
            with self.lock:
                direction = -1 if self.model == 'dm' and self.status.get('mode') != 'fake_rs_pos_vel' else 1
                self.gripper_opening = max(0.0, min(90.0, direction * value / 5.0 * 90))
                self.gripper_at = time.monotonic()

    def _text(self, name, msg):
        with self.lock:
            setattr(self, name, msg.data[:4000])

    def state(self):
        now = time.monotonic()
        with self.lock:
            return {**self.status, 'mode': 'ros', 'control_mode': self.status.get('mode'),
                    'model': self.model, 'namespace': self.namespace, 'motion_allowed': self.allow_motion,
                    'positions': dict(self.positions), 'velocities': dict(self.velocities),
                    'motor_states':{name:{**value,'age_sec':now-value['received_at']} for name,value in self.motor_states.items()},
                    'joint_sample_time': self.joint_at,
                    'gripper_opening_mm': self.gripper_opening if now-self.gripper_at < 2 else None,
                    'joint_age_sec': now-self.joint_at if self.joint_at else None,
                    'status_age_sec': now-self.status_at if self.status_at else None,
                    'gesture': self.gesture, 'transcript': self.transcript,
                    'manual_motion': {**self.manual_motion, 'queued': len(self.manual_pending)},
                    'action_motion': self._action_state(now)}

    def _check(self, command):
        if command in {'safety_stop', 'stop_action'}:
            return
        if not self.allow_motion:
            raise BridgeError('Read-only bridge. Restart with --allow-motion to control the arm.', 403)
        # Disable remains usable when feedback has expired or the safety latch is active.
        if command == 'disable':
            return
        state = self.state()
        if state['joint_age_sec'] is None or state['joint_age_sec'] > 1.5 or state['status_age_sec'] is None or state['status_age_sec'] > 2:
            raise BridgeError('Feedback missing or stale; refusing command.', 409)
        if state['error_codes'] or state['state_machine'] in {'SAFETY_STOPPED', 'SAFETY_HOMING', 'SAFETY_FAULT', 'SAFE_HOMING', 'GRAVITY_COMP', 'FAULT', 'ERROR'}:
            raise BridgeError('Controller is faulted or safety-latched; recover at the robot first.', 409)
        # Original MotorPassthrough stops the end-pose loop before entering
        # LOWLEVEL_STREAMING. Motor position/velocity control remains active.
        lowlevel=state['state_machine']=='LOWLEVEL_STREAMING' and state.get('control_mode') in ('pos_vel','mit')
        if command not in {'enable'} and (not state['enabled'] or not (state['control_loop_active'] or lowlevel)):
            raise BridgeError('Controller is disabled or control loop is inactive.', 409)

    def _await(self, future, timeout, *, interruptible=False):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        deadline = time.monotonic() + timeout
        while not event.wait(.05):
            if interruptible and self.stop_event.is_set():
                raise BridgeError('运动已停止。', 409)
            if time.monotonic() >= deadline:
                raise BridgeError('ROS response timed out. Check controller status before retrying.', 504)
        if future.exception():
            raise BridgeError(str(future.exception()), 502)
        return future.result()

    def command(self, payload):
        args = validate_command(payload, self.model)
        command = args['command']
        self._check(command)
        if command == 'move_joint':
            return self._enqueue_manual(args)
        if command == 'run_action':
            return self._enqueue_action(args)
        if command == 'stop_action':
            command = 'safety_stop'
        independent = command in {'safety_stop', 'disable'}
        if independent:
            self.cancel_manual()
        if not independent and not self.motion_lock.acquire(blocking=False):
            raise BridgeError('Another command is still running.', 409)
        try:
            if not independent:
                self.stop_event.clear()
            self._check(command)
            if command in self.triggers:
                client = self.triggers[command]
                if not client.wait_for_service(timeout_sec=.8):
                    raise BridgeError(f'ROS service unavailable: {command}', 503)
                response = self._await(client.call_async(self.Trigger.Request()), 30 if command == 'safe_home' else 5)
                return {'success': bool(response.success), 'message': response.message}
            if command == 'gripper':
                return self._set_gripper(args['opening_mm'])
            return self._execute_trajectory(args)
        finally:
            if not independent:
                self.motion_lock.release()

    def _set_gripper(self, opening, *, interruptible=False, verify=False):
        self._check('gripper')
        if not self.gripper_client.wait_for_service(timeout_sec=.8):
            raise BridgeError('Gripper service unavailable.', 503)
        if interruptible and self.stop_event.is_set():
            raise BridgeError('动作已停止。', 409)
        request = self.SetGripper.Request()
        with self.lock:
            hardware = self.status.get('mode') != 'fake_rs_pos_vel'
        request.position = opening / 90 * (-5.0 if hardware else 5.0)
        request.max_effort = 1.0
        response = self._await(self.gripper_client.call_async(request), 8, interruptible=interruptible)
        if response.success and verify:
            until = time.monotonic()+5
            while time.monotonic() < until:
                if self.stop_event.is_set():
                    raise BridgeError('动作已停止。', 409)
                self._check('gripper')
                with self.lock:
                    if time.monotonic()-self.gripper_at < .5 and self.gripper_opening is not None and abs(self.gripper_opening-opening) < 1:
                        break
                self.stop_event.wait(.03)
            else:
                raise BridgeError('夹爪反馈未到达目标，已中止后续动作。', 504)
        return {'success': bool(response.success), 'message': f'Gripper result: {response.reached_position:.3f} rad'}

    def _action_state(self, now):
        result = dict(self.action_motion)
        if result['status'] == 'running' and result.get('duration_sec'):
            elapsed = min(result.get('step_duration_sec', 0), max(0, now-result.get('step_started_at', now)))
            result['progress'] = min(.99, (result.get('completed_sec', 0)+elapsed)/result['duration_sec'])
        return result

    def _enqueue_action(self, args):
        with self.lock:
            if self.action_thread or self.manual_thread or self.manual_pending:
                raise BridgeError('当前有运动正在执行，请先停止或等待完成。', 409)
            self._ensure_original_available()
            title, operations = prepare_action(args['action'], [self.positions[name] for name in JOINTS], args['speed'], self.gripper_opening or 0)
            if not self.motion_lock.acquire(blocking=False):
                raise BridgeError('当前有其他动作正在执行。', 409)
            self.stop_event.clear()
            self.action_sequence += 1
            self.action_motion = {'status': 'running', 'sequence': self.action_sequence,
                                  'action': args['action'], 'title': title, 'speed': args['speed'],
                                  'step': 0, 'total_steps': operations[0]['total_steps'], 'progress': 0,
                                  'duration_sec': sum(step['duration'] for step in operations), 'completed_sec': 0,
                                  'step_started_at': time.monotonic(), 'step_duration_sec': 0, 'error': '', 'owns_node': False}
            self.action_thread = threading.Thread(target=self._action_loop, args=(operations, args['speed']), name='web-preset-action', daemon=True)
            try:
                self.action_thread.start()
            except Exception:
                self.action_thread = None
                self.motion_lock.release()
                raise
            return {'success': True, 'accepted': True, 'sequence': self.action_sequence,
                    'action_motion': self._action_state(time.monotonic()), 'message': f'{title}已开始执行。'}

    def _action_loop(self, operations, speed):
        status, error = 'completed', ''
        try:
            for index, operation in enumerate(operations):
                if self.stop_event.is_set():
                    raise BridgeError('动作已停止。', 409)
                self._check('run_action')
                with self.lock:
                    self.action_motion.update(step=index+1, phase=operation['kind'], step_started_at=time.monotonic(), step_duration_sec=0 if operation['kind'] == 'original' else operation['duration'])
                if operation['kind'] == 'original':
                    self._play_action_frames(operation)
                elif operation['kind'] == 'move':
                    self._execute_trajectory({'positions': operation['positions'], 'speed': speed})
                elif operation['kind'] == 'gripper':
                    result = self._set_gripper(operation['opening_mm'], interruptible=True, verify=True)
                    if not result['success']:
                        raise BridgeError('夹爪未完成，动作已中止。', 409)
                    with self.lock:
                        remaining = operation['duration']-(time.monotonic()-self.action_motion['step_started_at'])
                    self._action_wait(max(0, remaining))
                else:
                    self._action_wait(operation['duration'])
                if operation['kind'] != 'original':
                    with self.lock:
                        self.action_motion['completed_sec'] += operation['duration']
        except Exception as exc:
            status = 'stopped' if self.stop_event.is_set() else 'error'
            error = '' if status == 'stopped' else str(exc)
            if status == 'error' and self.action_motion.get('owns_node'):
                with self.lock:
                    self.action_motion.update(stop_origin='web_action_monitor',stop_reason=error)
                try:
                    self.command({'command': 'safety_stop'})
                except Exception:
                    pass
        finally:
            with self.lock:
                self.action_motion.update(status=status, error=error, owns_node=False)
                if status == 'completed':
                    self.action_motion['progress'] = 1.0
                self.action_thread = None
                self.motion_lock.release()

    def _ensure_original_available(self):
        for name in JOINTS:
            if self.node.count_publishers(f'/{self.namespace}/joints/{name}/cmd/pos_vel') > 1:
                raise BridgeError('检测到其他关节控制节点，请先停止命令行舞蹈、手势或跟随节点，再从网页执行。', 409)

    def _play_action_frames(self, program):
        """Run the unchanged original ROS node; RViz shows its actual feedback."""
        import yaml
        with self.lock:
            action = self.action_motion['action']
            speed = self.action_motion['speed']
            is_fake = self.status.get('mode') == 'fake_rs_pos_vel'
        self._ensure_original_available()
        kind = action if action in ('greeting', 'dance') else 'action'
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(self.domain_id)
        # A dedicated process owns only this invocation; no ROS daemon, desktop
        # RViz, existing CLI node or project configuration is stopped/modified.
        with tempfile.TemporaryDirectory(prefix='rebotarm-original-action-') as temporary:
            options = ['--ros-args', '-p', f'arm_namespace:={self.namespace}']
            if kind in ('greeting', 'dance'):
                package = f'rebotarm_{kind}'
                cfg = json.loads(json.dumps(_action_module.config(package, f'{kind}.yaml')))
                settings = cfg[kind]
                settings['speed']['time_scale'] /= speed
                settings['speed']['joint_vlim'] *= speed
                settings['gripper']['speed'] *= speed
                if is_fake:
                    settings['gripper']['open_pos'] = abs(settings['gripper']['open_pos'])
                if kind == 'dance':
                    settings['speed']['track_ramp_s'] /= speed
                path = Path(temporary) / 'action.yaml'
                path.write_text(yaml.safe_dump(cfg))
                options += ['-p', f'config_path:={path}', '-p', 'auto_start:=false',
                            '-p', f'rate_hz:={(60. if kind == "dance" else 30.) * speed}']
                if kind == 'dance':
                    options += ['-p', 'track_speed:=3.0', '-p',
                                f'tracks_path:={_action_module.ROOT}/src/rebotarm_dance/config/dance_tracks.yaml']
            else:
                options += ['-p', f'rate_hz:={30. * speed}',
                            '-p', f'entry_duration:={1.5 / speed}', '-p', f'reset_duration:={2. / speed}',
                            '-p', f'vlim:={5. * speed}', '-p', f'rock_vlim:={5. * speed}',
                            '-p', f'figure8_vlim:={5. * speed}', '-p', 'reset_pose:=zeros', '-p',
                            f'tracks_path:={_action_module.ROOT}/src/rebotarm_gesture_actions/config/gesture_tracks.yaml']
            invoked = time.monotonic()
            with (Path(temporary) / 'node.log').open('w+') as logfile:
                process = subprocess.Popen(['/usr/bin/python3', str(Path(__file__).with_name('action_worker.py')), kind, *options], env=env,
                                           stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
                with self.lock:
                    self.action_motion['owns_node'] = True
                try:
                    triggered_at, discovered = None, None
                    feedback=_worker_module.Feedback()
                    while True:
                        self._action_wait(.02)
                        if process.poll() is not None:
                            logfile.seek(0)
                            detail = logfile.read()[-1500:]
                            raise BridgeError(f'原项目动作节点提前退出：{detail}', 502)
                        publisher = self.original_play_pub if kind == 'action' else self.original_start_pubs[kind]
                        if triggered_at is None and publisher.get_subscription_count() > 0:
                            if discovered is None:
                                discovered = time.monotonic()
                            if time.monotonic() - discovered > .3:
                                if kind == 'action':
                                    track = {'rock': 'rock_push', 'paper': 'paper_figure8', 'heart': 'heart_draw', 'circle': 'paper_circle'}[action]
                                    publisher.publish(self.String(data=track))
                                else:
                                    publisher.publish(self.Bool(data=True))
                                triggered_at = time.monotonic()
                        with self.lock:
                            text, received = self.original_status.get(kind, ('', 0))
                            mode_same = (self.status.get('mode') == 'fake_rs_pos_vel') == is_fake
                        if not mode_same:
                            raise BridgeError('驱动模式已改变，原项目动作已停止。', 409)
                        now=time.monotonic()
                        # The worker copy is ordered and durable for this owned
                        # invocation, even if DDS misses its one-shot start status.
                        feedback.read(logfile,now)
                        if received > invoked:feedback.status(text,received,'ros')
                        with self.lock:
                            self.action_motion.update(node_status=feedback.text,status_source=feedback.source)
                        if feedback.finished:break
                        failure=feedback.timeout(now,invoked,triggered_at,program['duration'])
                        if failure:raise BridgeError(failure,504)
                        if feedback.began:
                            elapsed = now - feedback.started
                            # Progress describes the source script, not a browser animation.
                            index = min(len(program['frames']) - 1, int(elapsed / program['frames'][0]['duration']))
                            frame = program['frames'][index]
                            with self.lock:
                                self.action_motion.update(step=min(feedback.step or frame['step'],program['total_steps']), phase=feedback.phase or frame['phase'],
                                                          completed_sec=min(elapsed, program['duration']),
                                                          step_started_at=time.monotonic(), step_duration_sec=0)
                except Exception:
                    size=os.fstat(logfile.fileno()).st_size
                    tail=os.pread(logfile.fileno(),min(8192,size),max(0,size-8192)).decode(errors='replace')
                    with self.lock:self.action_motion['node_log_tail']=tail[-2400:]
                    raise
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=2)
        self._verify_joint_target(program['frames'][-1]['joints'])
        gripper_frames = [frame for frame in program['frames'] if frame['gripper_command']]
        if gripper_frames:
            target = gripper_frames[-1]['gripper']
            deadline = time.monotonic() + 5
            while True:
                self._check('run_action')
                if self.stop_event.is_set():
                    raise BridgeError('动作已停止。', 409)
                with self.lock:
                    reached = time.monotonic() - self.gripper_at < .5 and self.gripper_opening is not None and abs(self.gripper_opening - target) < 1
                if reached:
                    break
                if time.monotonic() > deadline:
                    raise BridgeError('夹爪反馈未到达原动作终点。', 504)
                self.stop_event.wait(.02)

    def _action_wait(self, duration):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            if self.stop_event.wait(min(.05, max(0, until-time.monotonic()))):
                raise BridgeError('动作已停止。', 409)
            self._check('run_action')

    def _enqueue_manual(self, args):
        with self.lock:
            if self.action_thread:
                raise BridgeError('预设动作正在执行，请先停止或等待完成。', 409)
            if self.stop_event.is_set() and self.manual_thread:
                raise BridgeError('正在停止上一段运动，请稍后再试。', 409)
            if self.motion_lock.locked() and not self.manual_thread:
                raise BridgeError('其他动作正在执行，请完成后再调节关节。', 409)
            self.manual_sequence += 1
            self.manual_pending[args['joint']] = {**args, 'sequence': self.manual_sequence}
            if not self.manual_thread:
                self.stop_event.clear()
                self.manual_thread = threading.Thread(target=self._manual_loop, name='web-manual-move', daemon=True)
                self.manual_thread.start()
            return {'success': True, 'accepted': True, 'sequence': self.manual_sequence,
                    'message': '目标已接收，将按设定速度移动。'}

    def _manual_loop(self):
        while True:
            with self.lock:
                if not self.manual_pending or self.stop_event.is_set():
                    self.manual_thread = None
                    return
                joint = next(iter(self.manual_pending))
                request = self.manual_pending.pop(joint)
                generation = self.manual_generation
                self.manual_motion = {'status': 'moving', 'sequence': request['sequence'],
                                      'joint': joint, 'target': request['position'], 'speed': request['speed']}
            try:
                if not self.motion_lock.acquire(blocking=False):
                    raise BridgeError('其他动作正在执行。', 409)
                try:
                    self._check('move_joint')
                    with self.lock:
                        positions = [self.positions[name] for name in JOINTS]
                    positions[JOINTS.index(joint)] = request['position']
                    if self.state().get('control_mode')=='pos_vel':
                        self._manual_pos_vel(request)
                    else:
                        self._execute_trajectory({'positions': positions, 'speed': request['speed']})
                    status, error = 'completed', ''
                finally:
                    self.motion_lock.release()
            except Exception as exc:
                status, error = 'error', str(exc)
            with self.lock:
                if generation == self.manual_generation:
                    self.manual_motion.update(status=status, error=error)
                    if status == 'error':
                        self.manual_pending.clear()

    def _manual_pos_vel(self,request):
        """Use the original DM motor target API, without a second interpolator.

        Publish only this joint's final target and motor velocity limit once.
        Original MotorPassthrough batches the group command; motor firmware
        executes the move. Browser polling never supplies intermediate targets.
        """
        self._ensure_original_available()
        publisher=self.fake_publishers[JOINTS.index(request['joint'])]
        deadline=time.monotonic()+.8
        while publisher.get_subscription_count()==0:
            self._check('move_joint')
            if self.stop_event.wait(.02):raise BridgeError('运动已停止。',409)
            if time.monotonic()>deadline:raise BridgeError('原 DM 关节位置／速度接口尚未就绪。',503)
        state=self.state()
        delta=abs(state['positions'][request['joint']]-request['position'])
        timeout=auto_duration(delta,request['speed'])+3
        with self.lock:
            self.manual_motion.update(duration_sec=max(.25,delta/request['speed']),transport='motor_pos_vel')
        self._check('move_joint')
        if self.stop_event.is_set():raise BridgeError('运动已停止。',409)
        message=self.JointPosVelCmd()
        message.pos,message.vlim=request['position'],request['speed']
        message.stamp=self.node.get_clock().now().to_msg()
        publisher.publish(message)
        deadline=time.monotonic()+timeout
        previous_sample=None
        settled=0
        try:
            while True:
                if self.stop_event.is_set():raise BridgeError('运动已停止。',409)
                self._check('move_joint')
                state=self.state()
                if state.get('control_mode')!='pos_vel':raise BridgeError('驱动模式已改变，关节运动已中止。',409)
                sample=state.get('joint_sample_time')
                if sample and sample!=previous_sample:
                    previous_sample=sample
                    position=state['positions'][request['joint']]
                    velocity=state['velocities'].get(request['joint'])
                    reached=abs(position-request['position'])<.01 and velocity is not None and abs(velocity)<.05
                    settled=settled+1 if reached else 0
                    if settled>=3:return
                if time.monotonic()>deadline:raise BridgeError('关节未在预计时间内到位，已请求原驱动停止。',504)
                self.stop_event.wait(.02)
        except Exception:
            # A one-shot motor target persists until replaced. On failed
            # feedback/timeout, stop it through the original independent service.
            if not self.stop_event.is_set():
                self.stop_event.set()
                try:
                    client=self.triggers['safety_stop']
                    if client.wait_for_service(timeout_sec=.8):self._await(client.call_async(self.Trigger.Request()),5)
                except Exception:pass
            raise

    def cancel_manual(self):
        self.stop_event.set()
        with self.lock:
            self.manual_generation += 1
            self.manual_pending.clear()
            if self.action_motion['status'] == 'running':
                self.action_motion.update(status='stopping')
            if self.manual_motion['status'] == 'moving':
                self.manual_motion.update(status='stopped', error='')
            handle = self.active_goal
        if handle is not None:
            handle.cancel_goal_async()

    def _execute_trajectory(self, args):
        with self.lock:
            start_positions = [self.positions[name] for name in JOINTS]
            is_fake = self.status.get('mode') == 'fake_rs_pos_vel'
        if 'speed' in args:
            delta = max(abs(a-b) for a, b in zip(start_positions, args['positions']))
            args = {**args, 'duration': auto_duration(delta, args['speed'])}
            with self.lock:
                if threading.current_thread() is self.manual_thread:
                    self.manual_motion['duration_sec'] = args['duration']
        if is_fake:
            return self._fake_trajectory(args)
        if not self.trajectory.wait_for_server(timeout_sec=.8):
            raise BridgeError('FollowJointTrajectory action unavailable.', 503)
        goal = self.FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        start = self.JointTrajectoryPoint()
        with self.lock:
            start.positions = [self.positions[name] for name in JOINTS]
        end = self.JointTrajectoryPoint()
        end.positions = args['positions']
        # Conservative speed bound for browser-originated trajectories.
        if any(abs(a-b) / args['duration'] > .5 for a,b in zip(start.positions, end.positions)):
            raise BridgeError('Requested average joint speed exceeds 0.5 rad/s; increase duration.', 409)
        end.time_from_start.sec, end.time_from_start.nanosec = divmod(round(args['duration'] * 1_000_000_000), 1_000_000_000)
        start.velocities = [0.0] * 6
        end.velocities = [0.0] * 6
        goal.trajectory.points = [start, end]
        future = self.trajectory.send_goal_async(goal)
        try:
            handle = self._await(future, 3, interruptible=True)
        except BridgeError:
            def cancel_late(completed):
                if not completed.exception():
                    late = completed.result()
                    if late.accepted:
                        late.cancel_goal_async()
            future.add_done_callback(cancel_late)
            raise
        if not handle.accepted:
            raise BridgeError('Controller rejected trajectory.', 409)
        try:
            with self.lock:
                self.active_goal = handle
            if self.stop_event.is_set():
                raise BridgeError('动作已停止。', 409)
            self._check('move_joints')
            result = self._await(handle.get_result_async(), args['duration'] + 5, interruptible=True)
        except BridgeError:
            handle.cancel_goal_async()
            raise
        finally:
            with self.lock:
                self.active_goal = None
        if result.result.error_code != 0 or result.status != 4:
            raise BridgeError(result.result.error_string or 'Trajectory canceled or failed.', 409)
        self._verify_joint_target(args['positions'])
        return {'success': True, 'message': 'Joint trajectory completed and feedback verified.'}

    def _fake_trajectory(self, args):
        """Use the repository FakeRsDriver's actual topic interface, never real hardware."""
        with self.lock:
            start = [self.positions[name] for name in JOINTS]
        if any(abs(a-b) / args['duration'] > .33 for a, b in zip(start, args['positions'])):
            raise BridgeError('Fake trajectory would exceed 0.5 rad/s peak; increase duration.', 409)
        deadline = time.monotonic() + args['duration']
        while True:
            self._check('move_joints')
            with self.lock:
                if self.status.get('mode') != 'fake_rs_pos_vel':
                    raise BridgeError('Fake driver mode changed; streaming aborted.', 409)
            if self.stop_event.is_set():
                raise BridgeError('Trajectory stopped.', 409)
            t = min(1.0, 1.0 - (deadline - time.monotonic()) / args['duration'])
            blend = t*t*(3-2*t)
            for publisher, a, b in zip(self.fake_publishers, start, args['positions']):
                msg = self.JointPosVelCmd()
                msg.pos = a + (b-a)*blend
                msg.vlim = args.get('speed', .5)
                msg.stamp = self.node.get_clock().now().to_msg()
                publisher.publish(msg)
            if t >= 1:
                break
            self.stop_event.wait(.02)
        self._verify_joint_target(args['positions'])
        return {'success': True, 'message': 'Fake driver trajectory completed and feedback verified.'}

    def _verify_joint_target(self, positions):
        until = time.monotonic()+2
        while time.monotonic() < until:
            self._check('move_joints')
            if self.stop_event.is_set():
                raise BridgeError('Trajectory stopped.', 409)
            with self.lock:
                if all(abs(self.positions[name]-goal) < .02 for name, goal in zip(JOINTS, positions)):
                    return
            self.stop_event.wait(.02)
        raise BridgeError('Joint feedback did not reach the target; subsequent motion was aborted.', 504)

    def close(self, extra_nodes=()):
        self.cancel_manual()
        for worker in (self.manual_thread, self.action_thread):
            if worker and worker is not threading.current_thread():
                worker.join(timeout=7)
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        for node in extra_nodes:
            node.destroy_node()
        self.node.destroy_node()
        self.rclpy.try_shutdown(context=self.context)


class ManagedSimulation:
    """Lazy, single-instance Fake driver with a private ROS context and domain.

    Opening the page requests start through POST; GETs and health checks never
    launch ROS. Only this repository's Fake driver can be started here.
    """

    def __init__(self, model='dm', namespace='rebotarm_web', domain_id=187):
        require_dm(model)
        self.model, self.namespace, self.domain_id = model, namespace, domain_id
        self.lock = threading.RLock()
        self.status, self.error = 'stopped', ''
        self.bridge = None
        self.worker = None
        self.stop_requested = threading.Event()

    def runtime(self):
        with self.lock:
            return {'managed': True, 'kind': 'fake', 'status': self.status,
                    'model': self.model, 'namespace': self.namespace,
                    'domain_id': self.domain_id, 'error': self.error}

    def state(self):
        with self.lock:
            state = self.bridge.state() if self.status == 'running' else DemoBridge(self.model, self.namespace).state()
            return {**state, 'runtime': self.runtime()}

    def start(self):
        with self.lock:
            if self.status in {'starting', 'running', 'stopping'}:
                return self.runtime()
            self.status, self.error = 'starting', ''
            self.stop_requested.clear()
            self.worker = threading.Thread(target=self._run, name='web-fake-session', daemon=True)
            self.worker.start()
            return self.runtime()

    def _run(self):
        bridge, fake, failure = None, None, ''
        try:
            from rclpy.parameter import Parameter
            from rebotarmcontroller.fake_rs_driver import FakeRsDriver

            bridge = RosBridge(self.model, self.namespace, True, domain_id=self.domain_id)
            manifest = json.loads((DIST / 'assets/models' / f'{self.model}.json').read_text())
            by_name = {j['name']: j for j in manifest['joints']}
            limits = [by_name[name][bound] for name in JOINTS for bound in ('lower', 'upper')]
            overrides = [Parameter('arm_namespace', value=self.namespace),
                         Parameter('joint_limits', value=limits),
                         Parameter('start_enabled', value=True),
                         # Original action nodes publish their configured 5/3 rad/s
                         # limits. Manual controls still enforce their 0.5 rad/s cap.
                         Parameter('max_joint_speed', value=5.0),
                         Parameter('max_gripper_speed', value=3.0),
                         Parameter('joint_state_rate', value=50.0),
                         Parameter('gripper_visual_joint_names', value=['finger_left'])]
            fake = FakeRsDriver(context=bridge.context, parameter_overrides=overrides)
            bridge.executor.add_node(fake)
            deadline = time.monotonic() + 10
            while not self.stop_requested.is_set():
                state = bridge.state()
                if state['joint_age_sec'] is not None and state['status_age_sec'] is not None:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('仿真已启动，但未收到 ROS 2 反馈。')
                self.stop_requested.wait(.05)
            if not self.stop_requested.is_set():
                with self.lock:
                    self.bridge, self.status = bridge, 'running'
                self.stop_requested.wait()
        except Exception as exc:
            failure = f'仿真启动失败：{exc}'
            print(failure, flush=True)
        finally:
            if bridge:
                bridge.stop_event.set()
                # Let an in-flight Fake stream observe cancellation before teardown.
                if bridge.motion_lock.acquire(timeout=5):
                    bridge.motion_lock.release()
                try:
                    bridge.close(extra_nodes=[fake] if fake else [])
                except Exception as exc:
                    failure = failure or f'仿真停止异常：{exc}'
            with self.lock:
                self.bridge = None
                self.status = 'error' if failure else 'stopped'
                self.error = failure

    def stop(self):
        with self.lock:
            if self.status in {'starting', 'running'}:
                self.status = 'stopping'
                self.stop_requested.set()
                if self.bridge:
                    self.bridge.stop_event.set()
            return self.runtime()

    def command(self, payload):
        with self.lock:
            bridge = self.bridge if self.status == 'running' else None
        if bridge is None:
            raise BridgeError('本机仿真尚未就绪，请等待启动或在网页中重试。', 409)
        return bridge.command(payload)

    def close(self):
        self.stop()
        if self.worker:
            self.worker.join(timeout=12)


class ManagedStudio(ManagedSimulation):
    """One explicit target shared by all pages and tabs; hardware never auto-starts."""
    def __init__(self,model='dm',namespace='rebotarm_web',domain_id=187):
        self.model=model
        self.simulation=ManagedSimulation(model,namespace,domain_id)
        self.hardware=_hardware_module.HardwareSession(RosBridge,DemoBridge,BridgeError,
                         domain_id=domain_id+1 if domain_id<232 else domain_id-1)
        self.target='fake'
        self.lock=threading.RLock()
        self.switching_to=None
        self.switch_error=''
        self.pending_device=''
        self.switch_worker=None
        self.prepare_switch=lambda:None
        self.closing=False

    @property
    def session(self):return self.hardware if self.target=='hardware' else self.simulation

    @property
    def bridge(self):return self.session.bridge

    def runtime(self):
        with self.lock:
            runtime=self.session.runtime()
            return {**runtime,'switching_to':self.switching_to,'switch_error':self.switch_error,
                    **({'status':'stopping'} if self.switching_to else {})}

    def state(self):
        with self.lock:
            result=self.session.state()
            return {**result,'runtime':self.runtime(),
                    **({'motion_allowed':False} if self.switching_to else {})}

    def start(self):
        with self.lock:
            if self.target=='hardware' or self.switching_to:
                raise BridgeError('真机仅由“真机连接”页启动；需要仿真时请点击“切换到仿真”。',409)
            return self.simulation.start()

    def stop(self):return self.session.stop()

    def command(self,payload):
        with self.lock:
            bridge=self.bridge
            if self.switching_to and (not isinstance(payload,dict) or payload.get('command') not in ('safety_stop','stop_action','disable')):
                raise BridgeError('正在切换控制对象，请等待连接完成。',409)
            if bridge is None or (self.session.runtime()['status']!='running' and not (isinstance(payload,dict) and payload.get('command') in ('safety_stop','stop_action','disable'))):
                raise BridgeError('当前控制对象尚未连接，请在网页中连接或启动。',409)
        return bridge.command(payload)

    def hardware_state(self):
        state=self.hardware.state()
        return {'available':True,'active_target':self.target,'switching_to':self.switching_to,
                'error':self.switch_error,'connection':self.hardware.runtime(),
                'devices':_hardware_module.serial_devices(),
                'feedback':state if state.get('mode')=='ros' else None}

    def connect_hardware(self,device=''):
        with self.lock:
            if self.target=='hardware' and self.hardware.runtime()['status'] in ('running','starting') and not self.switching_to:
                if device and device not in (self.hardware.device,str(Path(self.hardware.device).resolve())):
                    raise BridgeError('请先断开当前真机，再选择另一个串口。',409)
                return self.hardware_state()
        selection=self.hardware.select(device)
        self._switch('hardware',selection)
        return self.hardware_state()

    def switch_simulation(self):
        self._switch('fake')
        return self.hardware_state()

    def _switch(self,target,selection=None):
        with self.lock:
            if self.closing:raise BridgeError('网页服务正在关闭。',409)
            if self.switching_to:
                if self.switching_to==target and (target=='fake' or self.pending_device==selection['path']):return
                raise BridgeError('正在切换控制对象，请稍候。',409)
            if target=='fake' and self.target=='fake' and self.simulation.runtime()['status']=='running':return
            self.switching_to,self.switch_error=target,''
            self.pending_device=selection['path'] if selection else ''
            old=self.session
            if old.bridge:
                old.bridge.allow_motion=False
                old.bridge.cancel_manual()
            if old is self.hardware:
                old.stop()
            self.switch_worker=threading.Thread(target=self._switch_run,args=(old,target,selection),name='web-control-target',daemon=True)
            self.switch_worker.start()

    def _switch_run(self,old,target,selection):
        try:
            self.prepare_switch()
            old.close()
            if old.worker and old.worker.is_alive():raise RuntimeError('原控制对象尚未停止，未启动新的驱动。')
            with self.lock:
                if self.closing:return
                self.target=target
                if target=='hardware':self.hardware.start(selection)
                else:self.simulation.start()
        except Exception as exc:
            with self.lock:self.switch_error=str(exc)
        finally:
            with self.lock:self.switching_to=None;self.pending_device=''

    def disconnect_hardware(self):
        with self.lock:
            if self.switching_to:raise BridgeError('正在切换控制对象，请等待完成后再断开。',409)
        self.hardware.stop()
        self.prepare_switch() if self.target=='hardware' else None
        return self.hardware_state()

    def close(self):
        with self.lock:self.closing=True
        if self.switch_worker:self.switch_worker.join(timeout=25)
        self.hardware.close()
        self.simulation.close()


class WebServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, bridge, allowed_origins=()):
        super().__init__(address, partial(Handler, directory=str(DIST)))
        self.bridge = bridge
        self.stream_stop = threading.Event()
        port = self.server_address[1]
        self.allowed_hosts = {f'127.0.0.1:{port}', f'localhost:{port}'}
        self.allowed_origins = {f'http://{host}' for host in self.allowed_hosts} | set(allowed_origins)
        self.rviz = RvizSession(bridge, self.allowed_origins)
        self.features = _feature_module.FeatureSession(bridge)
        if isinstance(bridge,ManagedStudio):
            bridge.prepare_switch = self._prepare_switch

    def _prepare_switch(self):
        self.features.stop_all()
        self.rviz.close()
        if self.rviz.worker and self.rviz.worker.is_alive():
            raise BridgeError('旧 RViz 界面仍在退出，未切换控制对象。',409)

    def server_close(self):
        self.stream_stop.set()
        if hasattr(self, 'features'):
            self.features.close()
        if hasattr(self, 'rviz'):
            self.rviz.close()
        super().server_close()


class Handler(SimpleHTTPRequestHandler):
    server_version = 'reBotArmStudio/1.0'

    def _allowed(self):
        if self.headers.get('Host') not in self.server.allowed_hosts:
            return False
        origin = self.headers.get('Origin')
        return not origin or origin in self.server.allowed_origins

    def end_headers(self):
        origin = self.headers.get('Origin')
        if origin in self.server.allowed_origins:
            self.send_header('Access-Control-Allow-Origin', origin)
            self.send_header('Vary', 'Origin')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'same-origin')
        # HTML and its ES modules must not come from different local releases.
        # In particular, a cached controller script must never outlive its UI.
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def _json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        if not self._allowed():
            return self._json({'success': False, 'message': 'Origin or Host not allowed'}, 403)
        self.send_response(204)
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        if not self._allowed():
            return self._json({'success': False, 'message': 'Origin or Host not allowed'}, 403)
        path = urlsplit(self.path).path
        if path == '/api/joints/stream':
            return self._stream_joints()
        if path == '/api/state':
            return self._json({**self.server.bridge.state(), 'rviz': self.server.rviz.state(), 'interaction': self.server.features.state()})
        if path == '/api/hardware':
            return self._json(self.server.bridge.hardware_state() if isinstance(self.server.bridge,ManagedStudio) else {'available':False,'message':'当前服务未启用本机连接管理。'})
        if path == '/api/features':
            return self._json(self.server.features.state())
        if path == '/api/features/camera':
            return self._stream_camera()
        if path in {'/api/rviz/status', '/api/rviz/session'}:
            return self._json(self.server.rviz.state(credentials=path.endswith('/session')))
        if path == '/api/runtime':
            bridge = self.server.bridge
            return self._json(bridge.runtime() if isinstance(bridge, ManagedSimulation) else
                              {'managed': False, 'status': 'external'})
        if path.startswith('/api/'):
            return self._json({'success': False, 'message': 'Unknown API endpoint'}, 404)
        return super().do_GET()

    def _stream_joints(self):
        # Read-only, bounded-rate feedback stream. Normal Host/Origin checks run before this method.
        state = self.server.bridge.state()
        if state.get('mode') != 'ros':
            return self._json({'success': False, 'message': 'ROS 2 is not connected'}, 409)
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.close_connection = True
        self.connection.settimeout(3)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        last_sample, last_write = None, 0
        try:
            while not self.server.stream_stop.is_set():
                state = self.server.bridge.state()
                if state.get('mode') != 'ros':
                    self.wfile.write(b'event: unavailable\ndata: {}\n\n')
                    self.wfile.flush()
                    break
                sample_time = state.get('joint_sample_time')
                if sample_time and sample_time != last_sample:
                    frame = {key: state.get(key) for key in ('model', 'namespace', 'positions', 'gripper_opening_mm', 'joint_age_sec')}
                    frame['sample_time'] = sample_time
                    self.wfile.write(('event: joints\ndata: ' + json.dumps(frame, allow_nan=False, separators=(',', ':')) + '\n\n').encode())
                    self.wfile.flush()
                    last_sample, last_write = sample_time, time.monotonic()
                elif time.monotonic()-last_write > 1:
                    self.wfile.write(b': heartbeat\n\n')
                    self.wfile.flush()
                    last_write = time.monotonic()
                self.server.stream_stop.wait(1/30)
        except (OSError, ValueError):
            pass  # Closing a tab/reconnecting ends only that tab's read-only stream.

    def do_POST(self):
        if not self._allowed():
            return self._json({'success': False, 'message': 'Origin or Host not allowed'}, 403)
        path = urlsplit(self.path).path
        if path in {'/api/features/frame', '/api/features/audio'}:
            return self._feature_binary(path)
        if path not in {'/api/command', '/api/actions/preview', '/api/features/start', '/api/features/stop', '/api/features/text', '/api/hardware/connect', '/api/hardware/disconnect', '/api/hardware/simulation', '/api/runtime/start', '/api/runtime/stop', '/api/rviz/start', '/api/rviz/stop'}:
            return self._json({'success': False, 'message': 'Unknown API endpoint'}, 404)
        if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
            return self._json({'success': False, 'message': 'Content-Type must be application/json'}, 415)
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if length <= 0 or length > 16384:
                raise BridgeError('JSON body must be between 1 and 16384 bytes', 413)
            payload = json.loads(self.rfile.read(length))
            if path.startswith('/api/hardware/'):
                if not isinstance(self.server.bridge,ManagedStudio):
                    raise BridgeError('当前服务未启用本机连接管理。',409)
                if not isinstance(payload,dict) or set(payload)-({'device'} if path.endswith('/connect') else set()) or not isinstance(payload.get('device',''),str):
                    raise BridgeError('真机连接只接受列表中的串口设备，不接受其他启动参数。')
                result=(self.server.bridge.connect_hardware(payload.get('device','')) if path.endswith('/connect') else self.server.bridge.disconnect_hardware() if path.endswith('/disconnect') else self.server.bridge.switch_simulation())
                return self._json({'success':True,**result})
            if path.startswith('/api/features/'):
                if self.server.bridge.state().get('runtime',{}).get('switching_to'):
                    raise BridgeError('正在切换控制对象，请等待连接完成。',409)
                if not isinstance(payload, dict):
                    raise BridgeError('请选择有效功能。')
                if path.endswith('/text'):
                    if set(payload) != {'text'} or self.server.features.states['agent']['status'] != 'running':
                        raise BridgeError('请先开启文字与 MCP 功能。')
                    result = self.server.features.text(payload['text'])
                    return self._json({'success': True, **result})
                if set(payload) - {'feature','speed'} or not isinstance(payload.get('feature'),str) or payload['feature'] not in _feature_module.FEATURES:
                    raise BridgeError('请选择有效功能；不接受脚本或自定义启动参数。')
                result = self.server.features.start(payload['feature'], payload.get('speed',1)) if path.endswith('/start') else self.server.features.stop(payload['feature'])
                return self._json({'success': True, **result})
            if path == '/api/actions/preview':
                if not isinstance(payload, dict) or set(payload) - {'action', 'speed', 'positions', 'opening_mm'}:
                    raise BridgeError('预览仅接受动作名、倍率和初始姿态。')
                args = validate_command({'command': 'run_action', 'action': payload.get('action'), 'speed': payload.get('speed', 1)}, 'dm')
                positions = validate_command({'command': 'move_joints', 'positions': payload.get('positions')}, 'dm')['positions']
                opening = number(payload.get('opening_mm', 0), 'opening_mm', 0, 90)
                title, operations = prepare_action(args['action'], positions, args['speed'], opening)
                return self._json({'success': True, 'title': title, **operations[0]})
            if path.startswith('/api/rviz/'):
                if payload != {}:
                    raise BridgeError('RViz 启停接口仅接受空 JSON 对象。')
                try:
                    rviz = self.server.rviz.start() if path.endswith('/start') else self.server.rviz.stop()
                except ValueError as exc:
                    raise BridgeError(str(exc), 409)
                return self._json({'success': True, 'rviz': rviz})
            if path != '/api/command':
                if not isinstance(self.server.bridge, ManagedSimulation):
                    raise BridgeError('当前服务不管理本机仿真。', 409)
                if payload != {}:
                    raise BridgeError('仿真启动和停止接口仅接受空 JSON 对象。')
                if path.endswith('/stop'):
                    self.server.features.stop_all()
                    self.server.rviz.stop()
                runtime = self.server.bridge.start() if path.endswith('/start') else self.server.bridge.stop()
                return self._json({'success': True, 'runtime': runtime})
            result = self.server.bridge.command(payload)
            if isinstance(payload,dict) and payload.get('command') in {'safety_stop','disable'}:
                self.server.features.stop_all()
            return self._json(result, 200 if result.get('success') else 409)
        except BridgeError as exc:
            return self._json({'success': False, 'message': str(exc)}, exc.status)
        except (ValueError, UnicodeDecodeError) as exc:
            return self._json({'success': False, 'message': str(exc) or 'Invalid JSON body'}, 400)
        except Exception as exc:
            self.log_error('Bridge failure: %s', exc)
            return self._json({'success': False, 'message': '本机 ROS 服务暂不可用，请重新连接或重新启动仿真。'}, 502)

    def _feature_binary(self, path):
        try:
            expected = 'image/jpeg' if path.endswith('/frame') else 'audio/pcm'
            if self.headers.get('Content-Type','').split(';')[0] != expected:
                raise BridgeError(f'Content-Type must be {expected}',415)
            length = int(self.headers.get('Content-Length','0'))
            if not 1 <= length <= 350000:
                raise BridgeError('输入过大或为空。',413)
            data = self.rfile.read(length)
            if path.endswith('/frame'):
                self.server.features.frame(data)
                return self._json({'success':True})
            return self._json({'success':True, **self.server.features.audio(data)})
        except BridgeError as exc:
            return self._json({'success':False,'message':str(exc)},exc.status)
        except ValueError as exc:
            return self._json({'success':False,'message':str(exc)},409)
        except Exception as exc:
            self.log_error('Feature input failed: %s',exc)
            return self._json({'success':False,'message':'本地识别失败，请重新开启该功能。'},502)

    def _stream_camera(self):
        if not self.server.features.state()['vision']:
            return self._json({'success':False,'message':'摄像头识别尚未开启。'},409)
        self.send_response(200)
        self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
        self.send_header('Connection','close')
        self.end_headers()
        self.close_connection = True
        self.connection.settimeout(3)
        last = -1
        try:
            while not self.server.stream_stop.is_set() and self.server.features.state()['vision']:
                sequence,jpeg = self.server.features.image()
                if jpeg and sequence != last:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                    self.wfile.flush()
                    last = sequence
                self.server.stream_stop.wait(1/30)
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--ros', action='store_true', help='Subscribe to ROS 2; requires sourced ROS environment')
    parser.add_argument('--managed-sim', action='store_true', help='Start the local Fake driver when the webpage opens')
    parser.add_argument('--domain-id', type=int, default=187, help='Private ROS domain for managed simulation')
    parser.add_argument('--model', choices=('dm',), default='dm')
    parser.add_argument('--namespace', default=None, help='Match the namespace used by your ROS controller')
    parser.add_argument('--allow-motion', action='store_true', help='Explicitly allow sending control commands')
    parser.add_argument('--allow-origin', action='append', default=[], help='Additional exact browser origin; never use a wildcard')
    args = parser.parse_args()
    namespace = (args.namespace or ('rebotarm_web' if args.managed_sim else 'rebotarm')).strip('/')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', namespace):
        parser.error('Invalid ROS namespace')
    if not 1 <= args.port <= 65535:
        parser.error('Port must be within 1..65535')
    if not 0 <= args.domain_id <= 232:
        parser.error('ROS domain ID must be within 0..232')
    if args.managed_sim and (args.ros or args.allow_motion):
        parser.error('--managed-sim manages its own Fake driver; do not combine with --ros or --allow-motion')
    for origin in args.allow_origin:
        parsed = urlsplit(origin)
        if parsed.scheme not in ('http', 'https') or not parsed.netloc or parsed.path or parsed.query or parsed.fragment or parsed.username:
            parser.error('--allow-origin requires an exact http(s) origin without a path')
    if args.allow_motion and not args.ros:
        parser.error('--allow-motion requires --ros')
    try:
        bridge = (ManagedStudio(args.model, namespace, args.domain_id) if args.managed_sim else
                  RosBridge(args.model, namespace, args.allow_motion) if args.ros else DemoBridge(args.model, namespace))
    except ImportError as exc:
        parser.exit(2, f'ROS dependency unavailable: {exc}\nUse /usr/bin/python3 after sourcing /opt/ros/<distro>/setup.bash and install/setup.bash.\n')
    server = WebServer(('127.0.0.1', args.port), bridge, args.allow_origin)
    print(f'reBot Arm Studio: http://127.0.0.1:{args.port}', flush=True)
    print(f'Mode: {"managed Fake simulation" if args.managed_sim else "ROS 2" if args.ros else "browser demo"} | Model: {args.model.upper()} | Namespace: /{namespace} | Motion: {args.allow_motion}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        bridge.close()


if __name__ == '__main__':
    main()

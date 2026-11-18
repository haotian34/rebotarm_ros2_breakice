"""Preset input, whole-program validation, mutual exclusion and failure handling."""
import importlib.util
import json
from pathlib import Path
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor

spec = importlib.util.spec_from_file_location('studio_actions_server', Path(__file__).resolve().parents[1] / 'server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)
CATALOG = json.loads((server.DIST / 'actions.json').read_text())['actions']


def wait_for(predicate):
    until = time.monotonic()+3
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('Action did not finish')


def stub_bridge():
    bridge = server.RosBridge.__new__(server.RosBridge)
    bridge.model = 'dm'
    bridge.lock, bridge.motion_lock = threading.RLock(), threading.Lock()
    bridge.stop_event = threading.Event()
    bridge.manual_pending, bridge.positions = {}, dict.fromkeys(server.JOINTS, 0.)
    bridge.manual_thread = bridge.action_thread = bridge.active_goal = None
    bridge.manual_generation = bridge.action_sequence = 0
    bridge.manual_motion = {'status': 'idle'}
    bridge.action_motion = {'status': 'idle', 'sequence': 0, 'progress': 0}
    bridge.gripper_opening = 0
    bridge._check = lambda command: None
    bridge._ensure_original_available = lambda: None
    return bridge


class PresetTests(unittest.TestCase):
    def test_external_cli_controller_is_rejected_without_stopping_it(self):
        bridge = stub_bridge()
        def external():
            raise server.BridgeError('external controller', 409)
        bridge._ensure_original_available = external
        bridge.cancel_manual = lambda: self.fail('Web must not stop a preexisting CLI controller')
        with self.assertRaises(server.BridgeError):
            bridge.command({'command': 'run_action', 'action': 'dance'})
        self.assertIsNone(bridge.action_thread)
        self.assertFalse(bridge.stop_event.is_set())

    def test_only_fixed_actions_and_speed_are_accepted(self):
        for action in CATALOG:
            args = server.validate_command({'command': 'run_action', 'action': action, 'speed': 1.}, 'dm')
            self.assertEqual(args['action'], action)
        for payload in ({'action': '../../shell'}, {'action': []}, {'action': 'unknown'}, {'action': 'rock', 'speed': 1.1}, {'action': 'rock', 'speed': True}, {'action': 'rock', 'steps': []}, {'action': 'rock', 'shell': 'x'}):
            with self.assertRaises(server.BridgeError):
                server.validate_command({'command': 'run_action', **payload}, 'dm')
        with self.assertRaises(server.BridgeError):
            server.validate_command({'command': 'stop_action', 'action': 'x'}, 'dm')

    def test_speed_scales_time_and_limits_without_changing_a_single_frame(self):
        manifest = json.loads((server.DIST / 'assets/models/dm.json').read_text())
        limits = {joint['name']: joint for joint in manifest['joints']}
        starts = [[0.]*6, [limits[name]['lower'] for name in server.JOINTS], [limits[name]['upper'] for name in server.JOINTS]]
        for action in CATALOG:
            for start in starts:
                _, baseline = server.prepare_action(action, start, 1.)
                original = baseline[0]
                self.assertEqual(original['frames'][-1]['joints'], [0.]*6)
                for speed in (.25, .5, .75):
                    _, operations = server.prepare_action(action, start, speed)
                    program = operations[0]
                    self.assertEqual(len(program['frames']), len(original['frames']))
                    self.assertAlmostEqual(program['duration'], original['duration']/speed)
                    for frame, reference in zip(program['frames'], original['frames']):
                        for key in ('joints', 'gripper', 'gripper_command', 'step', 'phase'):
                            self.assertEqual(frame[key], reference[key])
                        self.assertAlmostEqual(frame['duration'], reference['duration']/speed)
                        for value, vlim in zip(frame['vlim'], reference['vlim']):
                            self.assertAlmostEqual(value, vlim*speed)

    def test_step_failure_aborts_remaining_motion_and_frees_lock(self):
        bridge = stub_bridge()
        calls = []
        def fail(args):
            calls.append(args)
            raise server.BridgeError('controller rejected')
        bridge._play_action_frames = fail
        bridge._set_gripper = lambda *args, **kwargs: self.fail('Gripper ran after a failed move')
        result = bridge.command({'command': 'run_action', 'action': 'dance'})
        self.assertTrue(result['accepted'])
        wait_for(lambda: bridge.action_thread is None)
        self.assertEqual(bridge.action_motion['status'], 'error')
        self.assertEqual(len(calls), 1)
        self.assertIn('controller rejected', bridge.action_motion['error'])
        self.assertFalse(bridge.motion_lock.locked())

    def test_concurrent_clicks_run_only_one_action_and_manual_cannot_overlap(self):
        bridge = stub_bridge()
        entered = threading.Event()
        calls = []
        def hold(args):
            calls.append(args)
            entered.set()
            bridge.stop_event.wait(3)
            raise server.BridgeError('stopped')
        bridge._play_action_frames = hold
        def start(_):
            try:
                return bridge.command({'command': 'run_action', 'action': 'greeting'})['accepted']
            except server.BridgeError:
                return False
        try:
            with ThreadPoolExecutor(max_workers=8) as pool:
                results = list(pool.map(start, range(8)))
            self.assertEqual(sum(results), 1)
            self.assertTrue(entered.wait(1))
            with self.assertRaises(server.BridgeError):
                bridge.command({'command': 'move_joint', 'joint': 'joint1', 'position': .1})
            bridge.cancel_manual()
            wait_for(lambda: bridge.action_thread is None)
            self.assertEqual(bridge.action_motion['status'], 'stopped')
            self.assertEqual(len(calls), 1)
            self.assertFalse(bridge.motion_lock.locked())
        finally:
            bridge.stop_event.set()


if __name__ == '__main__':
    unittest.main()

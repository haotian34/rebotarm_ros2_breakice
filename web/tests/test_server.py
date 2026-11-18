"""Bridge validation and HTTP security checks; no ROS or hardware required."""
import http.client
import importlib.util
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('studio_server', Path(__file__).resolve().parents[1] / 'server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class ValidationTests(unittest.TestCase):
    def test_manual_duration_tracks_peak_speed_for_full_joint_range(self):
        for speed in (.05, .1, .3, .5):
            for delta in (.001, .1, 1, 3.14, 6.28):
                duration = server.auto_duration(delta, speed)
                self.assertLessEqual(1.5 * delta / duration, speed)
        self.assertGreater(server.auto_duration(3.14, .1), 30)

    def test_manual_target_validates_model_joint_and_speed(self):
        for model, position in [('dm', -.2)]:
            result = server.validate_command({'command': 'move_joint', 'joint': 'joint2', 'position': position, 'speed': .3}, model)
            self.assertEqual(result['position'], position)
        for patch in ({'joint': 'joint7'}, {'position': .1}, {'speed': 0}, {'speed': .6}, {'speed': True}, {'position': float('nan')}):
            with self.assertRaises(server.BridgeError):
                server.validate_command({'command': 'move_joint', 'joint': 'joint2', 'position': -.2, 'speed': .3, **patch}, 'dm')

    def test_valid_dm_limits(self):
        for model, shoulder in [('dm', -1)]:
            result = server.validate_command({'command': 'move_joints', 'positions': [0, shoulder, shoulder, 0, 0, 0], 'duration': 4}, model)
            self.assertEqual(result['positions'][1], shoulder)

    def test_other_models_are_rejected_before_ros_initialization(self):
        for factory in (server.DemoBridge, server.ManagedSimulation, server.RosBridge):
            with self.assertRaises(server.BridgeError):
                factory('rs')
        with self.assertRaises(server.BridgeError):
            server.validate_command({'command': 'enable'}, 'rs')

    def test_out_of_range_wrong_direction_and_bad_length(self):
        for positions in [[0, 1, -1, 0, 0, 0], [0, -1, -1, 0, 0, 4], [0]*5]:
            with self.assertRaises(server.BridgeError):
                server.validate_command({'command': 'move_joints', 'positions': positions}, 'dm')

    def test_nonfinite_bool_and_invalid_duration(self):
        for value in [float('nan'), float('inf'), True, '1', None, -1, 91]:
            with self.assertRaises(server.BridgeError):
                server.validate_command({'command': 'gripper', 'opening_mm': value}, 'dm')
        with self.assertRaises(server.BridgeError):
            server.validate_command({'command': 'move_joints', 'positions': [0]*6, 'duration': .1}, 'dm')

    def test_unmapped_commands_and_payloads(self):
        for payload in [{}, [], {'command': 'shell'}, {'command': []}]:
            with self.assertRaises(server.BridgeError):
                server.validate_command(payload, 'dm')

    def test_ros_permission_and_freshness_gate(self):
        bridge = server.RosBridge.__new__(server.RosBridge)
        bridge.allow_motion = False
        with self.assertRaises(server.BridgeError) as ctx:
            bridge._check('enable')
        self.assertEqual(ctx.exception.status, 403)
        bridge._check('safety_stop')
        bridge.allow_motion = True
        bridge.state = lambda: {'joint_age_sec': 5, 'status_age_sec': 0, 'enabled': True}
        with self.assertRaises(server.BridgeError):
            bridge._check('move_joints')
        bridge._check('disable')
        healthy = {'joint_age_sec': .1, 'status_age_sec': .1, 'error_codes': [], 'enabled': True, 'control_loop_active': True, 'state_machine': 'READY'}
        bridge.state = lambda: healthy
        bridge._check('move_joints')
        for mutation in [{'state_machine': 'SAFETY_STOPPED'}, {'error_codes': ['motor fault']}, {'enabled': False}, {'control_loop_active': False}]:
            bridge.state = lambda m=mutation: {**healthy, **m}
            with self.assertRaises(server.BridgeError):
                bridge._check('move_joints')
        bridge.state=lambda:{**healthy,'state_machine':'LOWLEVEL_STREAMING','control_mode':'pos_vel','control_loop_active':False}
        bridge._check('move_joint');bridge._check('run_action')
        for mutation in ({'enabled':False},{'control_mode':'fake_rs_pos_vel'},{'state_machine':'IDLE'},{'state_machine':'SAFETY_FAULT'}):
            bridge.state=lambda m=mutation:{**healthy,'state_machine':'LOWLEVEL_STREAMING','control_mode':'pos_vel','control_loop_active':False,**m}
            with self.assertRaises(server.BridgeError):bridge._check('move_joint')


class HttpTests(unittest.TestCase):
    def test_features_are_idle_until_clicked_and_reject_custom_commands(self):
        status, body, _ = self.request('GET', '/api/features')
        self.assertEqual(status, 200)
        self.assertTrue(all(value['status'] == 'stopped' for value in json.loads(body)['features'].values()))
        for payload in ({'feature':'shell'}, {'feature':'gesture','command':'anything'}, {'feature':['gesture']}):
            status, _, _ = self.request('POST','/api/features/start',json.dumps(payload),{'Content-Type':'application/json'})
            self.assertEqual(status,400)
        status, body, _ = self.request('POST','/api/features/start','{"feature":"gesture"}',{'Content-Type':'application/json'})
        self.assertEqual(status,400)
        self.assertIn('当前机械臂尚未就绪',body.decode())
        status, _, _ = self.request('GET','/api/features/camera')
        self.assertEqual(status,409)

    def test_feature_inputs_validate_type_size_and_origin(self):
        for path in ('/api/features/frame','/api/features/audio'):
            status, _, _ = self.request('POST',path,b'x',{'Content-Type':'text/plain'})
            self.assertEqual(status,415)
            status, _, _ = self.request('POST',path,b'x',{'Content-Type':'image/jpeg','Origin':'https://elsewhere.invalid'})
            self.assertEqual(status,403)
        status, _, _ = self.request('POST','/api/features/frame',b'not jpeg',{'Content-Type':'image/jpeg'})
        self.assertEqual(status,409)
        status, _, _ = self.request('POST','/api/features/frame',b'x'*350001,{'Content-Type':'image/jpeg'})
        self.assertEqual(status,413)
        status, _, _ = self.request('POST','/api/features/text','{"text":"画爱心"}'.encode(),{'Content-Type':'application/json'})
        self.assertEqual(status,400)

    @classmethod
    def setUpClass(cls):
        cls.httpd = server.WebServer(('127.0.0.1', 0), server.DemoBridge())
        cls.thread = threading.Thread(target=cls.httpd.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.httpd.server_close()
        cls.thread.join()

    def request(self, method, path, body=None, headers=None):
        conn = http.client.HTTPConnection(*self.httpd.server_address, timeout=3)
        conn.request(method, path, body=body, headers=headers or {})
        response = conn.getresponse()
        data = response.read()
        status, returned_headers = response.status, dict(response.getheaders())
        conn.close()
        return status, data, returned_headers

    def test_static_and_state(self):
        status, body, headers = self.request('GET', '/')
        self.assertEqual(status, 200)
        self.assertIn('reBot Arm Studio'.encode(), body)
        self.assertIn(b'boot.js?v=', body)
        self.assertEqual(headers['Cache-Control'], 'no-store')
        for path in ('/boot.js?v=test', '/app.js?v=test', '/action-presets.js?v=test'):
            status, _, headers = self.request('GET', path)
            self.assertEqual(status, 200)
            self.assertEqual(headers['Cache-Control'], 'no-store')
        status, body, headers = self.request('GET', '/api/state')
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)['mode'], 'demo')
        self.assertFalse(json.loads(body)['motion_allowed'])
        self.assertEqual(headers['Cache-Control'], 'no-store')

    def test_demo_never_dispatches_motion(self):
        status, body, _ = self.request('POST', '/api/command', json.dumps({'command': 'enable'}), {'Content-Type': 'application/json'})
        self.assertEqual(status, 409)
        self.assertFalse(json.loads(body)['success'])

    def test_original_preview_is_read_only_and_speed_scaled(self):
        payload = {'action': 'heart', 'positions': [0]*6, 'speed': .5}
        status, body, _ = self.request('POST', '/api/actions/preview', json.dumps(payload), {'Content-Type': 'application/json'})
        self.assertEqual(status, 200)
        program = json.loads(body)
        self.assertEqual(program['kind'], 'original')
        self.assertEqual(program['frames'][-1]['joints'], [0]*6)
        self.assertEqual(program['total_steps'], 3)
        self.assertFalse(self.httpd.bridge.state()['motion_allowed'])
        for patch in ({'positions': [0, 1, 0, 0, 0, 0]}, {'speed': 2}, {'frames': []}, {'action': 'pick'}):
            status, _, _ = self.request('POST', '/api/actions/preview', json.dumps({**payload, **patch}), {'Content-Type': 'application/json'})
            self.assertEqual(status, 400)

    def test_rejects_external_origins_and_hosts(self):
        for headers in [{'Origin': 'https://untrusted.example'}, {'Origin': 'null'}, {'Host': 'untrusted.example'}]:
            status, _, _ = self.request('GET', '/api/state', headers=headers)
            self.assertEqual(status, 403)
        status, _, _ = self.request('POST', '/api/command', '{}', {'Content-Type': 'text/plain'})
        self.assertEqual(status, 415)

    def test_malformed_and_oversized_json(self):
        status, _, _ = self.request('POST', '/api/command', 'not-json', {'Content-Type': 'application/json'})
        self.assertEqual(status, 400)
        status, _, _ = self.request('POST', '/api/command', ' '*16385, {'Content-Type': 'application/json'})
        self.assertEqual(status, 413)

    def test_preflight_only_accepts_allowed_origin(self):
        origin = f'http://127.0.0.1:{self.httpd.server_address[1]}'
        status, _, headers = self.request('OPTIONS', '/api/command', headers={'Origin': origin})
        self.assertEqual(status, 204)
        self.assertEqual(headers['Access-Control-Allow-Origin'], origin)
        status, _, _ = self.request('OPTIONS', '/api/command', headers={'Origin': 'https://untrusted.example'})
        self.assertEqual(status, 403)

    def test_managed_start_is_post_only_and_has_fixed_payload(self):
        original = self.httpd.bridge
        managed = server.ManagedSimulation()
        self.httpd.bridge = managed
        try:
            with patch.object(managed, 'start', return_value={'managed': True, 'status': 'starting'}) as start:
                status, body, _ = self.request('GET', '/api/runtime')
                self.assertEqual(status, 200)
                self.assertEqual(json.loads(body)['status'], 'stopped')
                self.request('GET', '/api/state')
                self.request('GET', '/api/runtime/start')
                start.assert_not_called()
                status, _, _ = self.request('POST', '/api/runtime/start', '{"command":"arbitrary"}', {'Content-Type': 'application/json'})
                self.assertEqual(status, 400)
                start.assert_not_called()
                status, body, _ = self.request('POST', '/api/runtime/start', '{}', {'Content-Type': 'application/json'})
                self.assertEqual(status, 200)
                self.assertEqual(json.loads(body)['runtime']['status'], 'starting')
                start.assert_called_once()
        finally:
            self.httpd.bridge = original

    def test_rviz_reads_never_start_and_state_does_not_expose_credentials(self):
        with patch.object(self.httpd.rviz, 'start') as start:
            for path in ('/api/rviz/status', '/api/rviz/session'):
                status, body, _ = self.request('GET', path)
                self.assertEqual(status, 200)
                self.assertNotIn('password', json.loads(body))
            self.request('GET', '/api/rviz/start')
            start.assert_not_called()
        with patch.object(self.httpd.rviz, 'status', 'running'), patch.object(self.httpd.rviz, 'password', 'test-only'):
            _, body, _ = self.request('GET', '/api/state')
            self.assertNotIn('password', json.loads(body)['rviz'])

    def test_rviz_requires_ros_fixed_payload_and_trusted_origin(self):
        headers = {'Content-Type': 'application/json'}
        for payload, expected in [('{}', 409), ('{"command":"x"}', 400)]:
            status, _, _ = self.request('POST', '/api/rviz/start', payload, headers)
            self.assertEqual(status, expected)
        status, _, _ = self.request('GET', '/api/rviz/session', headers={'Origin': 'https://untrusted.example'})
        self.assertEqual(status, 403)
        with patch.object(self.httpd.rviz, 'stop') as stop:
            self.request('POST', '/api/runtime/stop', '{"bad":true}', headers)
            stop.assert_not_called()

    def test_joint_stream_requires_ros_and_rejects_untrusted_origin(self):
        status, _, _ = self.request('GET', '/api/joints/stream')
        self.assertEqual(status, 409)
        status, _, _ = self.request('GET', '/api/joints/stream', headers={'Origin': 'https://untrusted.example'})
        self.assertEqual(status, 403)

    def test_external_mode_does_not_offer_process_control(self):
        status, _, _ = self.request('POST', '/api/runtime/start', '{}', {'Content-Type': 'application/json'})
        self.assertEqual(status, 409)


if __name__ == '__main__':
    unittest.main()

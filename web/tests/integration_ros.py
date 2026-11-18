"""Optional real ROS 2 test with the repository's FakeRsDriver, no hardware.

Run with the system Python after sourcing ROS and install/setup.bash.
Always uses an isolated domain and loopback discovery.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '186'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOG_DIR'] = '/tmp/rebotarm-web-ros-test-logs'

import importlib.util
from pathlib import Path
import time

from rebotarmcontroller.fake_rs_driver import FakeRsDriver

spec = importlib.util.spec_from_file_location('studio_server', Path(__file__).resolve().parents[1] / 'server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


def wait_for(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('Timed out waiting for ROS feedback')


def test_model(model):
    bridge = server.RosBridge(model, 'rebotarm_rs', allow_motion=False)
    fake = FakeRsDriver()
    if model == 'dm':
        fake.joint_limits = [(-2.8, 2.8), (-3.14, 0), (-3.14, 0), (-1.87, 1.57), (-1.57, 1.57), (-3.14, 3.14)]
    bridge.executor.add_node(fake)
    try:
        wait_for(lambda: bridge.state()['joint_age_sec'] is not None and bridge.state()['status_age_sec'] is not None)
        state = bridge.state()
        assert state['mode'] == 'ros' and state['control_mode'] == 'fake_rs_pos_vel'
        assert len(state['positions']) >= 6
        try:
            bridge.command({'command': 'enable'})
            raise AssertionError('Read-only bridge accepted a motion command')
        except server.BridgeError as exc:
            assert exc.status == 403
        bridge.allow_motion = True
        assert bridge.command({'command': 'enable'})['success']
        wait_for(lambda: bridge.state()['enabled'])
        shoulder = -.2 if model == 'dm' else .2
        assert bridge.command({'command': 'move_joints', 'positions': [.1, shoulder, shoulder, .05, .05, .1], 'duration': 2})['success']
        assert abs(bridge.state()['positions']['joint2']-shoulder) < .02
        assert bridge.command({'command': 'gripper', 'opening_mm': 45})['success']
        wait_for(lambda: (bridge.state()['gripper_opening_mm'] or 0) > 40)
        assert bridge.command({'command': 'safety_stop'})['success']
        wait_for(lambda: bridge.state()['state_machine'] == 'SAFETY_STOPPED')
        try:
            bridge.command({'command': 'move_joints', 'positions': [0]*6, 'duration': 2})
            raise AssertionError('Safety latch failed to block trajectory')
        except server.BridgeError as exc:
            assert exc.status == 409
        assert bridge.command({'command': 'disable'})['success']
        print(f'PASS {model.upper()}: real ROS subscriptions, read-only gate, fake joint motion, gripper, safety latch, disable')
    finally:
        bridge.executor.remove_node(fake)
        bridge.close()
        fake.destroy_node()


if __name__ == '__main__':
    for model in ('dm',):
        test_model(model)

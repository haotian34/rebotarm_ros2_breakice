"""All original web actions on a private Fake driver; never hardware."""
import os
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOG_DIR'] = '/tmp/rebotarm-web-action-test-logs'
import importlib.util
import json
from pathlib import Path
import time

spec = importlib.util.spec_from_file_location('studio_server', Path(__file__).resolve().parents[1] / 'server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


def wait_for(predicate, timeout=20):
    until = time.monotonic()+timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('Action readiness/stop timed out')


def main():
    simulation = server.ManagedSimulation('dm', 'rebotarm_web_action_test', 195)
    try:
        simulation.start()
        wait_for(lambda: simulation.runtime()['status'] in {'running', 'error'})
        assert simulation.runtime()['status'] == 'running', simulation.runtime()
        catalog = json.loads((server.DIST/'actions.json').read_text())['actions']
        for action, speed in [(name, 1.) for name in catalog] + [('heart', .5)]:
            state = simulation.state()
            _, operations = server.prepare_action(action, [state['positions'][name] for name in server.JOINTS], speed, state['gripper_opening_mm'])
            program = operations[0]
            started = time.monotonic()
            accepted = simulation.command({'command': 'run_action', 'action': action, 'speed': speed})
            assert accepted['accepted'] and time.monotonic()-started < 1
            for payload in ({'command': 'run_action', 'action': action}, {'command': 'move_joint', 'joint': 'joint1', 'position': .1}, {'command': 'gripper', 'opening_mm': 40}):
                try:
                    simulation.command(payload)
                    raise AssertionError('Concurrent motion was accepted')
                except server.BridgeError as exc:
                    assert exc.status == 409
            peak, movement = 0, 0
            deadline = time.monotonic()+program['duration']+15
            while time.monotonic() < deadline:
                state = simulation.state()
                motion = state['action_motion']
                peak = max(peak, *(abs(v) for v in state['velocities'].values()))
                movement = max(movement, *(abs(v) for v in state['positions'].values()))
                if motion['status'] != 'running':
                    break
                time.sleep(.01)
            assert motion['status'] == 'completed', motion
            assert motion['progress'] == 1 and movement > .5
            assert peak <= 5*speed+.0001, peak
            assert all(abs(state['positions'][name]-target)<.02 for name,target in zip(server.JOINTS,program['frames'][-1]['joints']))
            commands = [frame for frame in program['frames'] if frame['gripper_command']]
            if commands:
                assert abs(state['gripper_opening_mm']-commands[-1]['gripper'])<1
            elapsed = time.monotonic()-started
            assert abs(elapsed-program['duration']) < 4, (elapsed,program['duration'])
            print(f'PASS {action} {speed}x: {len(program["frames"])} original frames, {elapsed:.2f}s, peak={peak:.3f}, final joints/gripper confirmed; overlap rejected', flush=True)
        simulation.command({'command': 'run_action', 'action': 'greeting', 'speed': 1})
        wait_for(lambda: simulation.state()['action_motion'].get('phase') == 'gripper')
        assert simulation.command({'command': 'stop_action'})['success']
        wait_for(lambda: simulation.state()['action_motion']['status'] == 'stopped')
        wait_for(lambda: simulation.state()['state_machine'] == 'SAFETY_STOPPED')
        stopped = simulation.state()
        time.sleep(.2)
        after = simulation.state()
        assert after['action_motion']['step'] == stopped['action_motion']['step']
        assert all(abs(after['positions'][name]-stopped['positions'][name])<.002 for name in server.JOINTS)
        assert abs(after['gripper_opening_mm']-stopped['gripper_opening_mm'])<1
        print('PASS stop: arm/gripper stop, no later frame, safety latch active', flush=True)
    finally:
        simulation.close()
    print('PASS: private ROS session cleaned up; no hardware accessed', flush=True)


if __name__ == '__main__':
    main()

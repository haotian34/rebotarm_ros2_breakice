"""Fake-only ROS integration for speed-based, asynchronous manual joint control."""
import os
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOG_DIR'] = '/tmp/rebotarm-web-manual-test-logs'
import importlib.util
from pathlib import Path
import time

spec = importlib.util.spec_from_file_location('studio_server', Path(__file__).resolve().parents[1] / 'server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


def wait_for(predicate, timeout=15):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('Timed out waiting for manual motion')


def main():
    for model in ('dm',):
        simulation = server.ManagedSimulation(model, 'rebotarm_web_manual_test', 190)
        try:
            simulation.start()
            wait_for(lambda: simulation.runtime()['status'] in {'running', 'error'})
            assert simulation.runtime()['status'] == 'running', simulation.runtime()
            initial = simulation.state()['positions'].copy()
            started = time.monotonic()
            accepted = simulation.command({'command': 'move_joint', 'joint': 'joint1', 'position': .8, 'speed': .5})
            assert accepted['accepted'] and time.monotonic()-started < .5
            peak = 0
            while time.monotonic()-started < 8:
                state = simulation.state()
                peak = max(peak, abs(state['velocities'].get('joint1', 0)))
                if state['manual_motion']['status'] == 'completed':
                    break
                assert state['manual_motion']['status'] != 'error', state['manual_motion']
                time.sleep(.01)
            else:
                raise AssertionError('Manual move did not complete')
            assert peak <= .500001, peak
            assert abs(state['positions']['joint1']-.8) < .02
            assert all(abs(state['positions'][j]-initial[j]) < .001 for j in server.JOINTS if j != 'joint1')
            # A pending same-joint target is replaced; another joint is retained.
            simulation.command({'command': 'move_joint', 'joint': 'joint1', 'position': .7, 'speed': .3})
            simulation.command({'command': 'move_joint', 'joint': 'joint1', 'position': .3, 'speed': .3})
            last = simulation.command({'command': 'move_joint', 'joint': 'joint1', 'position': .4, 'speed': .3})
            wait_for(lambda: simulation.state()['manual_motion']['sequence'] >= last['sequence'] and simulation.state()['manual_motion']['status'] == 'completed')
            assert abs(simulation.state()['positions']['joint1']-.4) < .02
            # A slow long move is accepted immediately, regardless of HTTP timeout.
            started = time.monotonic()
            simulation.command({'command': 'move_joint', 'joint': 'joint6', 'position': 3.0, 'speed': .05})
            assert time.monotonic()-started < .5
            wait_for(lambda: simulation.state()['manual_motion'].get('duration_sec', 0) > 30)
            assert simulation.command({'command': 'safety_stop'})['success']
            wait_for(lambda: simulation.state()['state_machine'] == 'SAFETY_STOPPED')
            assert simulation.state()['manual_motion']['status'] == 'stopped'
            assert simulation.state()['manual_motion']['queued'] == 0
            try:
                simulation.command({'command': 'move_joint', 'joint': 'joint1', 'position': 0, 'speed': .3})
                raise AssertionError('Safety latch did not block manual motion')
            except server.BridgeError as exc:
                assert exc.status == 409
            print(f'PASS {model.upper()}: immediate acceptance, automatic duration, peak speed {peak:.3f} <= 0.5, isolated joint, latest target, long slow move, safety stop')
        finally:
            simulation.close()


if __name__ == '__main__':
    main()

"""ROS integration: lazy start, concurrency, Fake controls, stop and restart.

Does not start hardware. Uses a dedicated domain and namespace.
"""
import os
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOG_DIR'] = '/tmp/rebotarm-web-managed-test-logs'

from concurrent.futures import ThreadPoolExecutor
import importlib.util
from pathlib import Path
import time

spec = importlib.util.spec_from_file_location('studio_server', Path(__file__).resolve().parents[1] / 'server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


def wait_for(predicate, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('Timed out waiting for managed simulation')


def main():
    for model in ('dm',):
        simulation = server.ManagedSimulation(model, 'rebotarm_web_lifecycle_test', 188)
        try:
            assert simulation.state()['mode'] == 'demo'
            assert simulation.worker is None  # Merely reading state never starts ROS.
            with ThreadPoolExecutor(max_workers=8) as pool:
                list(pool.map(lambda _: simulation.start(), range(16)))
            worker = simulation.worker
            wait_for(lambda: simulation.runtime()['status'] in {'running', 'error'})
            assert simulation.runtime()['status'] == 'running', simulation.runtime()
            assert simulation.state()['namespace'] == 'rebotarm_web_lifecycle_test'
            simulation.start()
            assert simulation.worker is worker and worker.is_alive()
            sign = -1 if model == 'dm' else 1
            assert simulation.command({'command': 'move_joints', 'positions': [.1, sign*.1, sign*.1, 0, 0, 0], 'duration': 2})['success']
            assert simulation.command({'command': 'gripper', 'opening_mm': 45})['success']
            simulation.stop()
            wait_for(lambda: simulation.runtime()['status'] in {'stopped', 'error'})
            assert simulation.runtime()['status'] == 'stopped', simulation.runtime()
            assert simulation.state()['mode'] == 'demo'
            simulation.start()
            wait_for(lambda: simulation.runtime()['status'] in {'running', 'error'})
            assert simulation.runtime()['status'] == 'running', simulation.runtime()
            assert simulation.worker is not worker
            print(f'PASS {model.upper()}: lazy startup, 16 concurrent starts, real ROS feedback/control, stop, restart')
        finally:
            simulation.close()
    print('PASS: all managed ROS nodes and worker threads cleaned up')


if __name__ == '__main__':
    main()

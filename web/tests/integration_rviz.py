"""Real RViz/Xvfb startup and TF updates on isolated DM Fake simulations."""
import os
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOG_DIR'] = '/tmp/rebotarm-rviz-ros-test-logs'
import http.client
import importlib.util
from pathlib import Path
import time
from tf2_msgs.msg import TFMessage

spec = importlib.util.spec_from_file_location('studio_server', Path(__file__).resolve().parents[1] / 'server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


def wait_for(predicate, timeout=30):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('RViz/TF readiness timed out')


def main():
    for model in ('dm',):
        simulation = server.ManagedSimulation(model, 'rebotarm_rviz_test', 191)
        rviz = server.RvizSession(simulation, {'http://127.0.0.1:8088'})
        processes = []
        try:
            simulation.start()
            wait_for(lambda: simulation.runtime()['status'] in {'running', 'error'})
            assert simulation.runtime()['status'] == 'running', simulation.runtime()
            stamps = []
            simulation.bridge.node.create_subscription(TFMessage, '/rebotarm_web_rviz/tf', lambda msg: stamps.append(time.monotonic()), 100)
            assert rviz.start()['status'] == 'starting'
            worker = rviz.worker
            rviz.start()
            assert rviz.worker is worker
            wait_for(lambda: rviz.state()['status'] in {'running', 'error'})
            assert rviz.state()['status'] == 'running', rviz.state()
            assert 'password' not in rviz.state()
            processes = rviz.processes.copy()
            wait_for(lambda: len(stamps) > 20)
            # WebSocket rejects foreign websites before VNC authentication.
            conn = http.client.HTTPConnection('127.0.0.1', rviz.state(credentials=True)['ws_port'], timeout=3)
            conn.request('GET', '/', headers={'Upgrade': 'websocket', 'Connection': 'Upgrade',
                         'Sec-WebSocket-Key': 'dGhlIHNhbXBsZSBub25jZQ==', 'Sec-WebSocket-Version': '13',
                         'Origin': 'https://untrusted.example'})
            response = conn.getresponse()
            assert response.status == 403, response.status
            response.read()
            conn.close()
            stamps.clear()
            simulation.command({'command': 'move_joint', 'joint': 'joint1', 'position': .3, 'speed': .3})
            wait_for(lambda: simulation.state()['manual_motion']['status'] == 'completed')
            hz = (len(stamps)-1)/(stamps[-1]-stamps[0])
            assert hz > 15, hz
            assert abs(simulation.state()['positions']['joint1']-.3) < .02
            print(f'PASS {model.upper()}: native RViz window, authenticated local display, origin rejection, live isolated TF {hz:.1f} Hz, manual motion', flush=True)
        finally:
            rviz.close()
            simulation.close()
        assert rviz.state()['status'] == 'stopped'
        assert all(process.poll() is not None for process in processes)
    print('PASS: all native display processes and Fake simulations cleaned up')


if __name__ == '__main__':
    main()

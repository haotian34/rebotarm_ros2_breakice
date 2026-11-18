"""Real read-only SSE feedback while moving isolated DM Fake drivers."""
import os
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOG_DIR'] = '/tmp/rebotarm-web-stream-test-logs'
import http.client
import importlib.util
import json
from pathlib import Path
import threading
import time

spec = importlib.util.spec_from_file_location('studio_server', Path(__file__).resolve().parents[1] / 'server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


def main():
    for model in ('dm',):
        simulation = server.ManagedSimulation(model, 'rebotarm_web_stream_test', 193)
        httpd = server.WebServer(('127.0.0.1', 0), simulation)
        thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        thread.start()
        connection = None
        try:
            simulation.start()
            until = time.monotonic()+20
            while simulation.runtime()['status'] == 'starting' and time.monotonic() < until:
                time.sleep(.03)
            assert simulation.runtime()['status'] == 'running', simulation.runtime()
            connection = http.client.HTTPConnection(*httpd.server_address, timeout=4)
            connection.request('GET', '/api/joints/stream')
            response = connection.getresponse()
            assert response.status == 200
            assert response.getheader('Content-Type').startswith('text/event-stream')
            accepted = simulation.command({'command': 'move_joint', 'joint': 'joint1', 'position': .5, 'speed': .3})
            assert accepted['accepted']
            samples, arrivals = [], []
            for _ in range(300):
                line = response.readline()
                if line.startswith(b'data: '):
                    sample = json.loads(line[6:])
                    assert sample['model'] == model
                    assert sample['namespace'] == 'rebotarm_web_stream_test'
                    assert sample['joint_age_sec'] < .2
                    assert set(server.JOINTS).issubset(sample['positions'])
                    samples.append(sample)
                    arrivals.append(time.monotonic())
                    if arrivals[-1]-arrivals[0] > 3:
                        break
            assert len(samples) > 60, len(samples)
            hz = (len(samples)-1)/(arrivals[-1]-arrivals[0])
            assert hz >= 20, hz
            assert all(a['sample_time'] < b['sample_time'] for a,b in zip(samples,samples[1:]))
            assert samples[-1]['positions']['joint1'] > .48
            assert max(b-a for a,b in zip(arrivals,arrivals[1:])) < .2
            print(f'PASS {model.upper()}: {hz:.1f} Hz real SSE feedback during Fake motion, monotonic timestamps, fresh joints, no polling requests', flush=True)
            response.close()
            connection.close()
        finally:
            if connection:
                connection.close()
            httpd.shutdown()
            httpd.server_close()
            thread.join(timeout=2)
            simulation.close()
    print('PASS: stream connections, HTTP server and Fake drivers cleaned up')


if __name__ == '__main__':
    main()

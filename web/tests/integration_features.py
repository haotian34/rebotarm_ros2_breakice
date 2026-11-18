"""Browser input through original ROS vision, follower, voice and MCP; Fake only."""
import os
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
os.environ['ROS_LOG_DIR']='/tmp/rebotarm-web-features-test-logs'
os.environ['MPLCONFIGDIR']='/tmp/rebotarm-web-features-mpl'
import importlib.util
import json
from urllib.request import Request,urlopen
from urllib.error import HTTPError
from pathlib import Path
import threading
import time
import cv2
import numpy as np

spec=importlib.util.spec_from_file_location('studio_server',Path(__file__).resolve().parents[1]/'server.py')
server=importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


def wait_for(predicate,timeout=30):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        result=predicate()
        if result:
            return result
        time.sleep(.05)
    raise AssertionError('Feature readiness timed out')


def main():
    simulation=server.ManagedSimulation('dm','rebotarm_web_features_test',197)
    server.Handler.log_message=lambda *args:None
    httpd=server.WebServer(('127.0.0.1',0),simulation)
    http_thread=threading.Thread(target=httpd.serve_forever,daemon=True);http_thread.start()
    features=httpd.features
    base='http://127.0.0.1:'+str(httpd.server_address[1])
    def post(path,data,kind='application/json'):
        if kind=='application/json':data=json.dumps(data).encode()
        with urlopen(Request(base+path,data=data,headers={'Content-Type':kind}),timeout=40) as response:
            return json.load(response)
    def start(feature,speed=1):return post('/api/features/start',{'feature':feature,'speed':speed})
    def stop(feature):return post('/api/features/stop',{'feature':feature})
    camera_stop=threading.Event()
    camera=None
    try:
        simulation.start()
        wait_for(lambda:simulation.runtime()['status'] in ('running','error'))
        assert simulation.runtime()['status']=='running',simulation.runtime()
        from geometry_msgs.msg import PointStamped
        from rclpy.qos import qos_profile_sensor_data
        palm_commands=[]
        simulation.bridge.node.create_subscription(PointStamped,'/rebotarm_web_features_test/palm_position',palm_commands.append,qos_profile_sensor_data)
        start('camera')
        jpeg=cv2.imencode('.jpg',np.full((480,640,3),170,np.uint8))[1].tobytes()
        def feed():
            while not camera_stop.wait(.08):
                if features.state()['vision']:
                    try:
                        post('/api/features/frame',jpeg,'image/jpeg')
                    except HTTPError as exc:
                        if exc.code!=409:raise
        camera=threading.Thread(target=feed,daemon=True)
        camera.start()
        def ready(name):
            state=features.state()['features'][name]
            assert state['status']!='error',state
            return state['status']=='running'
        wait_for(lambda:ready('camera'))
        sequence,frame=features.image()
        assert sequence>0 and cv2.imdecode(np.frombuffer(frame,np.uint8),cv2.IMREAD_COLOR).shape==(480,640,3)
        assert not simulation.state()['gesture'] or simulation.state()['gesture']=='none'
        assert not palm_commands, 'Read-only preview must not publish palm control input'
        with urlopen(base+'/api/features/camera',timeout=5) as response:
            assert 'multipart/x-mixed-replace' in response.headers['Content-Type']
            assert response.readline().strip()==b'--frame'
            assert response.readline().strip()==b'Content-Type: image/jpeg'
            length=int(response.readline().split(b':',1)[1]);response.readline()
            assert response.read(length).startswith(b'\xff\xd8')
        print('PASS: original MediaPipe inference publishes annotated JPEG and neutral gesture from browser input',flush=True)
        start('gesture',.5)
        wait_for(lambda:ready('gesture'))
        from std_msgs.msg import String
        publisher=simulation.bridge.node.create_publisher(String,'/rebotarm_web_features_test/gesture_cmd',10)
        # Inject only the classifier output; actual inference above remains active.
        msg=String();msg.data='heart'
        for _ in range(10):
            publisher.publish(msg);time.sleep(.06)
            if simulation.state()['action_motion']['sequence']:
                break
        wait_for(lambda:simulation.state()['action_motion']['status']=='running')
        assert simulation.state()['action_motion']['action']=='heart'
        assert simulation.state()['action_motion']['speed']==.5
        stop('gesture')
        assert simulation.state()['action_motion']['status']=='stopped',simulation.state()['action_motion']
        assert features.block_until==0 and features.action_sequence is None
        assert not features.processes
        print('PASS: stable gesture starts original heart action at 0.5x; closing feature cancels owned action and resets cooldown',flush=True)
        simulation.command({'command':'move_joints','positions':[0]*6,'duration':2})
        start('palm')
        wait_for(lambda:ready('palm'),45)
        assert features.state()['control_reserved']
        try:
            simulation.command({'command':'move_joint','joint':'joint1','position':.1,'speed':.3})
            raise AssertionError('Manual control overlapped follower')
        except server.BridgeError:
            pass
        wait_for(lambda:all(abs(simulation.state()['positions'][name]-target)<.025 for name,target in zip(server.JOINTS,[0,-.62,-.85,0,0,0])),15)
        assert features.processes['palm'].poll() is None
        wait_for(lambda:'已到起手位' in os.pread(features.logs['palm'].fileno(),65536,0).decode(errors='replace'),5)
        print('PASS: unchanged palm follower reaches original home; competing manual commands blocked',flush=True)
        camera_stop.set();camera.join(3)
        wait_for(lambda:features.state()['features']['palm']['status']=='error',10)
        assert '摄像头输入' in features.state()['features']['palm']['error'],features.state()
        assert not features.state()['control_reserved'] and not features.processes
        positions=simulation.state()['positions'];time.sleep(.25)
        assert all(abs(simulation.state()['positions'][name]-positions[name])<.01 for name in server.JOINTS)
        print('PASS: camera interruption stops owned follower, holds feedback position and releases control',flush=True)
        start('voice')
        wait_for(lambda:ready('voice'),55)
        try:
            features.audio(b'\0'*32000)
            raise AssertionError('Silence accepted')
        except ValueError as exc:
            assert '声音太小' in str(exc)
        sequence=simulation.state()['action_motion']['sequence']
        noise=np.random.default_rng(1).normal(0,.025,32000)
        try:
            features.audio((noise*32767).astype('<i2').tobytes())
            raise AssertionError('Non-speech audio accepted')
        except ValueError:
            pass
        assert simulation.state()['action_motion']['sequence']==sequence
        stop('voice')
        print('PASS: offline Whisper loads cached model; silence rejected; voice disables cleanly',flush=True)
        start('agent',.75)
        wait_for(lambda:ready('agent'),30)
        result=features.text('画圆')
        assert result['accepted'] and result['action']=='circle'
        assert simulation.state()['action_motion']['speed']==.75
        try:
            features.text('不要画爱心')
            raise AssertionError('Negative utterance accepted')
        except ValueError:
            pass
        simulation.bridge.cancel_manual()
        wait_for(lambda:not simulation.bridge.action_thread,10)
        stop('agent')
        assert not features.processes
        print('PASS: original MCP starts locally; text parser triggers matching action and rejects negation',flush=True)
    finally:
        camera_stop.set()
        if camera:
            camera.join(3)
        httpd.shutdown();httpd.server_close();http_thread.join(3)
        simulation.close()
    print('PASS: isolated Fake session and feature processes cleaned up; no hardware accessed',flush=True)


if __name__=='__main__':
    main()

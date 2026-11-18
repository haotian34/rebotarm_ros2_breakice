"""Exercise shared target switching over real ROS/HTTP using a Fake DM double.

The only driver launch is replaced with mock_dm_driver.py. No serial port opens.
"""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request,urlopen
from unittest.mock import patch

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
os.environ['ROS_LOG_DIR']='/tmp/rebotarm-web-target-test-logs'
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('target_test_server',ROOT/'server.py')
server=importlib.util.module_from_spec(spec);spec.loader.exec_module(server)


def wait(predicate,timeout=30):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        value=predicate()
        if value:return value
        time.sleep(.05)
    raise AssertionError('Target readiness timed out')


def main():
    studio=server.ManagedStudio(namespace='web_target_fake',domain_id=198)
    server.Handler.log_message=lambda *args:None
    httpd=server.WebServer(('127.0.0.1',0),studio)
    thread=threading.Thread(target=httpd.serve_forever,daemon=True);thread.start()
    base='http://127.0.0.1:'+str(httpd.server_address[1])
    def request(path,data=None):
        request=Request(base+path,data=None if data is None else json.dumps(data).encode(),headers={'Content-Type':'application/json'})
        with urlopen(request,timeout=40) as response:return json.load(response)
    original_popen=subprocess.Popen
    launched=[]
    def spawn(command,*args,**kwargs):
        if command[1:4]==['launch','rebotarm_bringup','driver.launch.py']:
            launched.append(command)
            command=['/usr/bin/python3',str(ROOT/'tests/mock_dm_driver.py'),studio.hardware.namespace]
        return original_popen(command,*args,**kwargs)
    try:
        assert request('/api/hardware')['connection']['status']=='stopped'
        assert studio.simulation.worker is None and studio.hardware.worker is None
        request('/api/runtime/start',{})
        wait(lambda:studio.simulation.runtime()['status']=='running')
        old=studio.bridge
        try:request('/api/hardware/connect',{'device':'/tmp/not-a-port'});raise AssertionError('Accepted non-device')
        except HTTPError as exc:assert exc.code==409
        assert studio.bridge is old and old.allow_motion
        print('PASS: diagnostics never connect; invalid device preserves running Fake',flush=True)
        native=httpd.rviz.state()['available']
        if native:
            request('/api/rviz/start',{})
            wait(lambda:httpd.rviz.state()['status'] in ('running','error'))
            assert httpd.rviz.state()['status']=='running',httpd.rviz.state()
        # /dev/null is only statted by the supervisor. The test double opens no device.
        with patch.object(studio.hardware,'select',return_value={'path':'/dev/null','refresh_group':None}),patch.object(server._hardware_module.subprocess,'Popen',side_effect=spawn):
            request('/api/hardware/connect',{'device':'test-double'})
            wait(lambda:studio.runtime()['status'] in ('running','error') and not studio.switching_to)
            assert studio.runtime()['status']=='running',studio.runtime()
            assert not studio.simulation.worker.is_alive()
            assert studio.runtime()['kind']=='hardware'
            diagnostic=wait(lambda:(lambda value:value if len(value.get('feedback',{}).get('motor_states',{}))==6 else None)(request('/api/hardware')))
            assert diagnostic['feedback']['control_mode']=='pos_vel'
            assert diagnostic['connection']['joint_count']==6
            assert len(launched)==1 and 'model:=dm' in launched[0]
            if native:assert httpd.rviz.state()['status']=='stopped'
            request('/api/hardware/connect',{})
            assert len(launched)==1
            print('PASS: one original driver launch request; six real ROS motor feedback topics; Fake stopped first',flush=True)
            from rebotarm_msgs.msg import JointPosVelCmd
            commands={name:[] for name in server.JOINTS}
            for name in commands:
                studio.bridge.node.create_subscription(JointPosVelCmd,f'/{studio.hardware.namespace}/joints/{name}/cmd/pos_vel',lambda msg,name=name:commands[name].append((msg.pos,msg.vlim)),10)
            time.sleep(.2)
            initial=studio.state()['positions'].copy()
            request('/api/command',{'command':'move_joint','joint':'joint1','position':.45,'speed':.3})
            peak=0
            deadline=time.monotonic()+10
            while studio.state()['manual_motion']['status'] not in ('completed','error') and time.monotonic()<deadline:
                peak=max(peak,abs(studio.state()['velocities'].get('joint1',0)))
                time.sleep(.01)
            wait(lambda:studio.state()['manual_motion']['status'] in ('completed','error'))
            assert studio.state()['manual_motion']['status']=='completed',studio.state()['manual_motion']
            assert abs(studio.state()['positions']['joint1']-.45)<.01
            assert .1<peak<=.300001,peak
            assert commands['joint1']==[(.45,.3)],commands
            assert all(not commands[name] for name in server.JOINTS if name!='joint1'),commands
            assert all(abs(studio.state()['positions'][name]-initial[name])<.001 for name in server.JOINTS if name!='joint1')
            assert studio.state()['state_machine']=='LOWLEVEL_STREAMING' and not studio.state()['control_loop_active']
            second=request('/api/command',{'command':'move_joint','joint':'joint4','position':.2,'speed':.1})
            wait(lambda:studio.state()['manual_motion']['sequence']==second['sequence'] and studio.state()['manual_motion']['status'] in ('completed','error'))
            assert studio.state()['manual_motion']['status']=='completed',studio.state()['manual_motion']
            assert commands['joint4']==[(.2,.1)],commands
            assert studio.state()['manual_motion']['transport']=='motor_pos_vel'
            request('/api/command',{'command':'gripper','opening_mm':30})
            wait(lambda:abs((studio.state()['gripper_opening_mm'] or 0)-30)<1)
            print('PASS: one final target per selected joint, motor speed cap, stationary other joints, no trajectory server; LOWLEVEL_STREAMING remains usable',flush=True)
            if native:
                request('/api/rviz/start',{})
                wait(lambda:httpd.rviz.state()['status'] in ('running','error'))
                assert httpd.rviz.state()['status']=='running',httpd.rviz.state()
            # The same feature manager must publish to the selected target's domain.
            import cv2
            import numpy as np
            jpeg=cv2.imencode('.jpg',np.full((240,320,3),170,np.uint8))[1].tobytes()
            request('/api/features/start',{'feature':'camera'})
            assert httpd.features.bridge is studio.hardware.bridge
            deadline=time.monotonic()+20
            while not httpd.features.image()[1] and time.monotonic()<deadline:
                with urlopen(Request(base+'/api/features/frame',data=jpeg,headers={'Content-Type':'image/jpeg'}),timeout=5) as response:response.read()
                time.sleep(.15)
            assert httpd.features.image()[1],'No annotated camera feedback on hardware target'
            request('/api/features/stop',{'feature':'camera'})
            print('PASS: shared native RViz and original MediaPipe camera follow the new target and ROS domain',flush=True)
            request('/api/command',{'command':'run_action','action':'heart','speed':.5})
            wait(lambda:studio.state()['action_motion']['status']=='running')
            wait(lambda:abs(studio.state()['positions']['joint2'])>.1)
            assert studio.state()['action_motion']['speed']==.5
            assert studio.state()['namespace']==studio.hardware.namespace
            request('/api/command',{'command':'stop_action'})
            wait(lambda:studio.state()['state_machine']=='SAFETY_STOPPED')
            print('PASS: same action endpoint runs original half-speed heart node on selected target; software stop reaches it',flush=True)
            # Simulate unplug detection, then ensure no automatic Fake fallback or driver retry.
            studio.hardware._device_present=lambda:False
            wait(lambda:studio.hardware.runtime()['status']=='error')
            assert request('/api/state')['runtime']['kind']=='hardware'
            time.sleep(.3);assert len(launched)==1
            assert not studio.simulation.worker.is_alive()
            print('PASS: disconnect detection stops the owned driver without reconnecting or falling back to Fake',flush=True)
            request('/api/hardware/simulation',{})
            wait(lambda:studio.runtime()['kind']=='fake' and studio.runtime()['status']=='running' and not studio.switching_to)
            assert studio.state()['namespace']=='web_target_fake'
            assert not studio.hardware.worker.is_alive()
            print('PASS: explicit switch restores the same Fake workspace after hardware cleanup',flush=True)
    finally:
        httpd.shutdown();httpd.server_close();thread.join(3);studio.close()
    print('PASS: all test processes closed; physical hardware was never opened',flush=True)


if __name__=='__main__':main()

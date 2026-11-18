"""Connection switching and driver lifecycle without opening hardware."""
import importlib.util
import http.client
import json
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import Mock,patch

spec=importlib.util.spec_from_file_location('hardware_studio_server',Path(__file__).resolve().parents[1]/'server.py')
server=importlib.util.module_from_spec(spec);spec.loader.exec_module(server)
hardware=server._hardware_module


def wait(predicate):
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.01)
    raise AssertionError('Target switch did not complete')


class StubSession:
    def __init__(self,kind,events,status='stopped'):
        self.kind,self.events,self.status=kind,events,status
        self.worker=None;self.device='/dev/test';self.bridge=None
        if status=='running':self.bridge=self.make_bridge()
    def make_bridge(self):return SimpleNamespace(allow_motion=True,cancel_manual=Mock(),command=Mock(return_value={'success':True}),state=lambda:{'mode':'ros','namespace':self.kind,'motion_allowed':True})
    def runtime(self):return {'kind':self.kind,'status':self.status,'managed':True,'model':'dm'}
    def state(self):return {**(self.bridge.state() if self.bridge else {'mode':'demo','namespace':self.kind}),'runtime':self.runtime()}
    def select(self,device):return {'path':device or '/dev/test'}
    def start(self,*args):self.events.append(self.kind+'.start');self.status='running';self.bridge=self.make_bridge()
    def stop(self):self.events.append(self.kind+'.stop');self.status='stopping';return self.runtime()
    def close(self):self.events.append(self.kind+'.close');self.bridge=None;self.status='stopped'


class TargetTests(unittest.TestCase):
    def setUp(self):
        self.studio=server.ManagedStudio()
        self.events=[]
        self.studio.simulation=StubSession('fake',self.events,'running')
        self.studio.hardware=StubSession('hardware',self.events)
        self.devices=patch.object(hardware,'serial_devices',return_value=[]);self.devices.start()
    def tearDown(self):self.studio.close();self.devices.stop()

    def test_failed_device_selection_preserves_running_simulation(self):
        self.studio.hardware.select=Mock(side_effect=server.BridgeError('Device missing',409))
        with self.assertRaises(server.BridgeError):self.studio.connect_hardware('/dev/test')
        self.assertEqual(self.studio.target,'fake');self.assertTrue(self.studio.bridge.allow_motion)
        self.assertFalse(self.events)

    def test_switch_blocks_motion_then_all_existing_commands_reuse_hardware_bridge(self):
        entered,release=threading.Event(),threading.Event()
        self.studio.prepare_switch=lambda:(entered.set(),release.wait(2))
        self.studio.connect_hardware('/dev/test');self.assertTrue(entered.wait(2))
        with self.assertRaises(server.BridgeError):self.studio.command({'command':'move_joint'})
        self.studio.connect_hardware('/dev/test')  # Repeated click cannot create a second driver.
        release.set();wait(lambda:not self.studio.switching_to)
        self.assertEqual(self.events,['fake.close','hardware.start'])
        bridge=self.studio.bridge
        for payload in ({'command':'move_joint','joint':'joint1','position':.1},{'command':'run_action','action':'heart','speed':.5},{'command':'gripper','opening_mm':30}):
            self.studio.command(payload);bridge.command.assert_called_with(payload)
        self.assertIs(bridge,self.studio.hardware.bridge)
        self.studio.connect_hardware('/dev/test')
        self.assertEqual(self.events.count('hardware.start'),1)
        with self.assertRaises(server.BridgeError):self.studio.start()  # Refresh must not reconnect hardware or start Fake.

    def test_return_to_simulation_stops_hardware_before_starting_fake(self):
        self.studio.target='hardware';self.studio.hardware.start();self.events.clear()
        self.studio.prepare_switch=lambda:self.events.append('prepare')
        self.studio.switch_simulation();wait(lambda:not self.studio.switching_to)
        self.assertEqual(self.events,['hardware.stop','prepare','hardware.close','fake.start'])
        self.assertEqual(self.studio.runtime()['kind'],'fake')

    def test_disconnect_starts_stopping_before_display_cleanup(self):
        self.studio.target='hardware';self.studio.hardware.start();self.events.clear()
        self.studio.prepare_switch=lambda:self.events.append('prepare')
        self.studio.disconnect_hardware()
        self.assertEqual(self.events,['hardware.stop','prepare'])
        self.assertEqual(self.studio.target,'hardware')


class DriverTests(unittest.TestCase):
    def run_driver(self,mode,stop_confirmed=True):
        session=hardware.HardwareSession(Mock(),server.DemoBridge,server.BridgeError)
        positions={f'joint{i}':0 for i in range(1,7)}|{'finger_left':0,'finger_right':0}
        bridge=SimpleNamespace(state=lambda:{'positions':positions,'joint_age_sec':.01,'status_age_sec':.01,'control_mode':mode},allow_motion=True,cancel_manual=Mock(),close=Mock())
        events=[];bridge.command=lambda request:(events.append('safety_stop') or {'success':stop_confirmed})
        session.factory.return_value=bridge
        process=Mock();process.returncode=None;process.pid=12345;process.poll.return_value=None
        process.wait.side_effect=lambda **kwargs:setattr(process,'returncode',0)
        session._device_present=lambda:True
        session.device='/dev/test';session.status='starting'
        proof=[]
        def tick(timeout):
            if session.status=='running':proof.append(True);return True
            return False
        session.stop_event=SimpleNamespace(wait=tick,is_set=lambda:False)
        clock=Mock(side_effect=[0,26]) if mode.startswith('fake') else Mock(return_value=0)
        with patch.object(hardware.subprocess,'Popen',return_value=process) as popen,patch.object(hardware.os,'killpg',side_effect=lambda *args:events.append('signal')) as killpg,patch.object(hardware.time,'monotonic',clock):
            session._run({})
        killpg.assert_called_once_with(process.pid,hardware.signal.SIGINT if stop_confirmed else hardware.signal.SIGKILL)
        command=popen.call_args.args[0]
        self.assertEqual(command[1:4],['launch','rebotarm_bringup','driver.launch.py'])
        self.assertIn('model:=dm',command)
        self.assertLess(events.index('safety_stop'),events.index('signal'))
        return session,proof

    def test_original_driver_feedback_with_gripper_joints_is_recognized(self):
        session,proof=self.run_driver('pos_vel')
        self.assertTrue(proof);self.assertEqual(session.joint_count,6)
        self.assertEqual(session.status,'stopped')

    def test_fake_feedback_cannot_mark_hardware_connected(self):
        session,proof=self.run_driver('fake_rs_pos_vel')
        self.assertFalse(proof);self.assertEqual(session.status,'error')
        self.assertIn('连接超时',session.error)

    def test_unconfirmed_stop_reports_error_instead_of_graceful_shutdown_homing(self):
        session,proof=self.run_driver('pos_vel',stop_confirmed=False)
        self.assertTrue(proof);self.assertEqual(session.status,'error')
        self.assertIn('停止反馈未确认',session.error)

    def test_only_discovered_accessible_free_devices_may_be_selected(self):
        session=hardware.HardwareSession(None,None,server.BridgeError)
        with patch.object(hardware,'serial_devices',return_value=[]):
            for value in ('','/tmp/command','/dev/ttyACM9'):
                with self.assertRaises(server.BridgeError):session.select(value)
        item={'path':'/dev/ttyACM0','device':'/dev/ttyACM0','accessible':False,'owners':[]}
        with patch.object(hardware,'serial_devices',return_value=[item]):
            with self.assertRaises(server.BridgeError) as caught:session.select()
            self.assertEqual(caught.exception.status,403)
            item['accessible']=True;item['owners']=[999]
            with self.assertRaises(server.BridgeError):session.select()
            item['owners']=[];self.assertEqual(session.select()['path'],'/dev/ttyACM0')


class HttpTests(unittest.TestCase):
    def test_diagnostics_are_read_only_and_connection_input_is_bounded(self):
        studio=server.ManagedStudio()
        httpd=server.WebServer(('127.0.0.1',0),studio)
        thread=threading.Thread(target=httpd.serve_forever,daemon=True);thread.start()
        def request(method,path,payload=None,origin=None):
            connection=http.client.HTTPConnection(*httpd.server_address,timeout=3)
            headers={'Content-Type':'application/json'}
            if origin:headers['Origin']=origin
            connection.request(method,path,None if payload is None else json.dumps(payload),headers)
            response=connection.getresponse();status=response.status;data=json.loads(response.read());connection.close()
            return status,data
        try:
            with patch.object(hardware,'serial_devices',return_value=[]),patch.object(studio.hardware,'start') as start:
                status,data=request('GET','/api/hardware')
                self.assertEqual(status,200);self.assertEqual(data['connection']['status'],'stopped')
                self.assertIsNone(studio.simulation.worker);start.assert_not_called()
                for payload in ([],{'device':[]},{'device':'/dev/ttyACM0','command':'shell'},{'domain_id':187}):
                    self.assertEqual(request('POST','/api/hardware/connect',payload)[0],400)
                self.assertEqual(request('POST','/api/hardware/connect',{},'https://other.invalid')[0],403)
                self.assertEqual(request('POST','/api/hardware/connect',{'device':'/tmp/anything'})[0],409)
                self.assertEqual(request('POST','/api/hardware/simulation',{'command':'anything'})[0],400)
                self.assertIsNone(studio.simulation.worker);start.assert_not_called()
        finally:
            httpd.shutdown();httpd.server_close();thread.join(3);studio.close()


if __name__=='__main__':unittest.main()

"""Run complete original actions while deliberately losing every ROS status.

Uses private Fake domain 201 only; never connects a physical controller.
"""
import importlib.util
import os
from pathlib import Path
import time

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
os.environ['ROS_LOG_DIR']='/tmp/rebotarm-action-feedback-test-logs'
spec=importlib.util.spec_from_file_location('action_feedback_server',Path(__file__).resolve().parents[1]/'server.py')
server=importlib.util.module_from_spec(spec);spec.loader.exec_module(server)


def wait(predicate,timeout=20):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.03)
    raise AssertionError('Timed out waiting for original action')


def main():
    simulation=server.ManagedSimulation(namespace='web_feedback_loss_test',domain_id=201)
    dropped=[]
    try:
        simulation.start();wait(lambda:simulation.runtime()['status'] in ('running','error'))
        assert simulation.runtime()['status']=='running',simulation.runtime()
        bridge=simulation.bridge
        bridge._original_status=lambda kind,message:dropped.append((kind,message.data))
        for action,speed in [('dance',1.),('greeting',.5),('heart',1.)]:
            before=len(dropped)
            simulation.command({'command':'run_action','action':action,'speed':speed})
            duration=simulation.state()['action_motion']['duration_sec']
            wait(lambda:simulation.state()['action_motion']['status'] in ('completed','error'),duration*2+15)
            state=simulation.state();motion=state['action_motion']
            assert motion['status']=='completed',motion
            assert motion['status_source']=='local_worker',motion
            assert state['state_machine']!='SAFETY_STOPPED',state
            assert len(dropped)>before,'Test did not intercept any DDS status'
            assert any('开始表演' in text or text.startswith('BUSY:') for _,text in dropped[before:]),dropped
            assert all(abs(state['positions'][name])<.02 for name in server.JOINTS)
            print(f'PASS: original {action} at {speed}x completed despite losing all ROS statuses; no false timeout/latch; final feedback verified',flush=True)
        simulation.command({'command':'run_action','action':'dance','speed':1.})
        wait(lambda:simulation.state()['action_motion'].get('step',0)>1)
        assert simulation.command({'command':'stop_action'})['success']
        wait(lambda:simulation.state()['action_motion']['status']=='stopped')
        assert simulation.state()['state_machine']=='SAFETY_STOPPED'
        print('PASS: explicit software stop still immediately latches and cancels the original worker',flush=True)
    finally:simulation.close()
    print('PASS: private Fake driver and original workers cleaned up; no hardware accessed',flush=True)


if __name__=='__main__':main()

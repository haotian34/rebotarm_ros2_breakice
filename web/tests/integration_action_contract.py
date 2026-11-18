"""DM POS_VEL contract with Fake-backed negative gripper feedback; no hardware."""
import os
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOG_DIR'] = '/tmp/rebotarm-web-action-contract-logs'
import importlib.util
from pathlib import Path
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState, JointPosVelCmd
from rebotarmcontroller.fake_rs_driver import FakeRsDriver

spec = importlib.util.spec_from_file_location('studio_contract_server', Path(__file__).resolve().parents[1] / 'server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


def wait_for(predicate, timeout=20):
    until = time.monotonic()+timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('Hardware contract timed out')


def main():
    simulation = server.ManagedSimulation('dm', 'rebotarm_hw_contract', 196)
    node = None
    try:
        simulation.start()
        wait_for(lambda: simulation.runtime()['status'] in {'running', 'error'})
        assert simulation.runtime()['status'] == 'running'
        bridge = simulation.bridge
        fake = next(item for item in bridge.executor.get_nodes() if isinstance(item, FakeRsDriver))
        node = Node('web_hardware_contract_controller', context=bridge.context)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        status_pub = node.create_publisher(ArmStatus, '/web_contract/arm_status', qos)
        gripper_pub = node.create_publisher(JointMotorState, '/web_contract/gripper', 10)
        bridge.node.destroy_subscription(bridge.subscriptions[1])
        bridge.node.destroy_subscription(bridge.subscriptions[2])
        bridge.node.create_subscription(ArmStatus, '/web_contract/arm_status', bridge._status, qos)
        bridge.node.create_subscription(JointMotorState, '/web_contract/gripper', bridge._gripper, 10)
        for subscription in list(fake.command_subscriptions):
            if subscription.topic_name == '/rebotarm_hw_contract/gripper/cmd/pos_vel':
                fake.destroy_subscription(subscription)
                fake.command_subscriptions.remove(subscription)
        requests = []

        def publish_feedback():
            status = ArmStatus()
            status.enabled = fake.enabled
            status.mode = 'dm_hardware_contract'
            status.control_loop_active = fake.enabled and not fake.safety_latched
            status.state_machine = fake.state_machine
            status_pub.publish(status)
            gripper = JointMotorState()
            gripper.position = -fake.gripper_position
            gripper_pub.publish(gripper)

        def gripper_command(message):
            assert -5 <= message.pos <= 0
            requests.append(message.pos)
            translated = JointPosVelCmd()
            translated.pos, translated.vlim = -message.pos, message.vlim
            fake._gripper_pos_vel_callback(translated)

        node.create_subscription(JointPosVelCmd, '/rebotarm_hw_contract/gripper/cmd/pos_vel', gripper_command, 100)
        node.create_timer(.03, publish_feedback)
        bridge.executor.add_node(node)
        wait_for(lambda: bridge.state()['control_mode'] == 'dm_hardware_contract')
        bridge.allow_motion = False
        try:
            bridge.command({'command': 'run_action', 'action': 'dance'})
            raise AssertionError('Read-only bridge accepted a hardware action')
        except server.BridgeError as exc:
            assert exc.status == 403
        bridge.allow_motion = True
        bridge.command({'command': 'run_action', 'action': 'dance', 'speed': 1})
        wait_for(lambda: bridge.state()['action_motion']['status'] != 'running', 35)
        assert bridge.state()['action_motion']['status'] == 'completed', bridge.state()['action_motion']
        assert requests and min(requests) == -5 and max(requests) == 0, requests
        assert abs(bridge.state()['gripper_opening_mm']) < 1
        print('PASS: complete original dance on hardware POS_VEL branch, negative DM gripper, final feedback, readonly rejection', flush=True)
        bridge.command({'command': 'run_action', 'action': 'greeting', 'speed': 1})
        wait_for(lambda: bridge.state()['action_motion'].get('phase') == 'gripper')
        assert bridge.command({'command': 'stop_action'})['success']
        wait_for(lambda: bridge.state()['action_motion']['status'] == 'stopped')
        count = len(requests)
        time.sleep(.2)
        assert len(requests) == count
        print('PASS: stop prevents all subsequent original joint/gripper commands and latches Fake controller', flush=True)
    finally:
        if node and simulation.bridge:
            simulation.bridge.executor.remove_node(node)
        if node:
            node.destroy_node()
        simulation.close()
    print('PASS: private mock hardware contract cleaned up; no hardware accessed', flush=True)


if __name__ == '__main__':
    main()

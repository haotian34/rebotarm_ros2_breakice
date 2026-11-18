"""Test-only DM protocol double backed by the original Fake driver; no serial IO."""
import json
from pathlib import Path
import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rebotarmcontroller.fake_rs_driver import FakeRsDriver


class PublisherAdapter:
    def __init__(self,publisher,transform):self.publisher,self.transform=publisher,transform
    def publish(self,message):self.transform(message);self.publisher.publish(message)


class MockDm(FakeRsDriver):
    def _gripper_pos_vel_callback(self,message):
        message.pos=-message.pos
        super()._gripper_pos_vel_callback(message)
    def _set_gripper(self,request,response):
        request.position=-request.position
        result=super()._set_gripper(request,response)
        result.reached_position=-result.reached_position
        return result


def main():
    namespace=sys.argv[1]
    manifest=json.loads((Path(__file__).resolve().parents[1]/'dist/assets/models/dm.json').read_text())
    joints={item['name']:item for item in manifest['joints']}
    limits=[joints[f'joint{i}'][bound] for i in range(1,7) for bound in ('lower','upper')]
    rclpy.init(args=[])
    node=MockDm(parameter_overrides=[Parameter('arm_namespace',value=namespace),Parameter('joint_limits',value=limits),Parameter('max_joint_speed',value=5.),Parameter('max_gripper_speed',value=3.)])
    def hardware_status(message):
        message.mode='pos_vel'
        # Unlike Fake, the real passthrough retains its motor mode at rest.
        if message.state_machine=='LOWLEVEL_STREAMING':node.motor_stream=True
        if message.state_machine=='IDLE' and getattr(node,'motor_stream',False):message.state_machine='LOWLEVEL_STREAMING'
        message.control_loop_active=message.enabled and message.state_machine!='LOWLEVEL_STREAMING'
    node.status_pub=PublisherAdapter(node.status_pub,hardware_status)
    def negative(message):message.position=-message.position;message.velocity=-message.velocity
    node.gripper_state_pub=PublisherAdapter(node.gripper_state_pub,negative)
    # Deliberately no FollowJointTrajectory server: DM manual control must use
    # the existing motor target topic, not the end-pose trajectory interpolator.
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()

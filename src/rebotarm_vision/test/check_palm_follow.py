"""Hardware-free ROS/kinematics integration check; run with system Python."""
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from rebotarm_vision.palm_follow_node import PalmFollowNode


def main():
    rclpy.init()
    node = PalmFollowNode()
    q = np.array([0., -.62, -.85, 0., 0., 0.])

    def joints():
        node.on_joints(JointState(name=node.names, position=q.tolist()))

    def palm(x=.5, y=.5, valid=1.):
        msg = PointStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = x, y, valid
        node.on_palm(msg)

    try:
        joints()
        node.tick()
        assert node.command is None
        for _ in range(5):
            palm()
        assert node.anchor is not None
        start = node.fk(node.measured_q())
        for _ in range(150):
            joints()
            palm(.8, .3)
            node.last_tick = time.monotonic() - 1 / 30
            previous = q.copy()
            node.tick()
            assert node.command is not None
            q = node.command[node.qidx].copy()
            assert np.max(np.abs(q - previous)) <= .3 / 30 + 1e-9
        joints()
        end = node.fk(node.measured_q())
        assert end[1] < start[1] - .03 and end[2] > start[2] + .015
        assert np.linalg.norm(end - start) <= .12
        palm(valid=0.)
        assert node.anchor is None and node.command is None
        for _ in range(5):
            joints()
            palm()
        assert np.linalg.norm(node.origin - end) < 1e-8
        node.palm_time = time.monotonic() - 1
        node.tick()
        assert node.anchor is None
        for _ in range(5):
            joints()
            palm()
        node.joint_times = {}
        node.tick()
        assert node.anchor is None
        print('PASS: Cartesian following, speed/range bounds, invalid palm, reacquisition, stale palm/joints')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

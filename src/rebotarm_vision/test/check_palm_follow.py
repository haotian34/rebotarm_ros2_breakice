"""Hardware-free ROS/kinematics integration check; run with system Python."""
import time
from unittest.mock import Mock
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from rebotarm_vision.palm_follow_node import PalmFollowNode


def check_delayed_feedback(node, reverse=False, rate=None):
    """100ms feedback lag plus finite actuator response must not cause stop/reacquire loops."""
    node.reset()
    node.phase = 'ready'
    node.pubs = [Mock() for _ in node.names]
    q = np.array([0., -.62, -.85, 0., 0., 0.])
    original_rate = node.cfg['rate_hz']
    if rate is not None:
        node.cfg['rate_hz'] = rate
    history = [q.copy() for _ in range(round(.1 * node.cfg['rate_hz']))]
    total = round(4.0 * node.cfg['rate_hz'])
    half_cycle = round(.8 * node.cfg['rate_hz'])
    original_reset, resets, outputs = node.reset, [], 0
    previous_y, delays, crossed = None, [], True

    def counted_reset():
        resets.append(True)
        original_reset()

    node.reset = counted_reset
    try:
        for k in range(total):
            node.on_joints(JointState(name=node.names, position=history.pop(0).tolist()))
            msg = PointStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            direction = 1 if (k // half_cycle) % 2 == 0 else -1
            if k % half_cycle == 0:
                crossed = False
            msg.point.x, msg.point.y, msg.point.z = ((.5 + .18 * direction, .5, 1.)
                                                   if reverse else (.75, .4, 1.))
            node.on_palm(msg)
            dt = 1 / node.cfg['rate_hz']
            node.last_tick = time.monotonic() - dt
            node.tick()
            outputs += int(node.command is not None)
            targets = np.array([pub.publish.call_args.args[0].pos if pub.publish.call_args else q[i]
                                for i, pub in enumerate(node.pubs)])
            speeds = np.array([pub.publish.call_args.args[0].vlim if pub.publish.call_args else .1
                               for pub in node.pubs])
            alpha = 1.0 - np.exp(-dt / .07)
            q += np.clip((targets - q) * alpha, -speeds * dt, speeds * dt)
            history.append(q.copy())
            state = node.measured_q()
            state[node.qidx] = q
            actual_y = node.fk(state)[1]
            if previous_y is not None and not crossed and (actual_y - previous_y) * direction > 1e-5:
                delays.append((k % half_cycle) * dt)
                crossed = True
            previous_y = actual_y
        assert not resets
        assert outputs >= total - node.cfg['acquire_frames'] - 2
        if reverse:
            assert len(delays) == 5
            assert np.median(delays[1:]) <= .22
            print('PASS: repeated direction reversals with delayed feedback, no resets')
        print('PASS: delayed actuator feedback, no stop/reacquire loop, matched motor speed')
    finally:
        node.reset = original_reset
        node.cfg['rate_hz'] = original_rate


def main():
    rclpy.init()
    node = PalmFollowNode()
    # Never publish motor commands from this check, even if a driver is running.
    node.pubs = [Mock() for _ in node.names]
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
        for _ in range(int(node.cfg['goto_home_duration'] * node.cfg['rate_hz']) + 2):
            joints()
            node.last_tick = time.monotonic() - 1 / node.cfg['rate_hz']
            node.tick()
        assert node.phase == 'ready'
        for pub in node.pubs:
            assert pub.publish.call_args.args[0].vlim == node.cfg['home_joint_speed']
        for _ in range(node.cfg['acquire_frames']):
            palm()
        assert node.anchor is not None
        start = node.fk(node.measured_q())
        previous_velocity = node.joint_velocity.copy()
        for k in range(150):
            joints()
            palm(.8, .3)
            node.last_tick = time.monotonic() - 1 / node.cfg['rate_hz']
            previous = q.copy()
            node.tick()
            assert node.command is not None
            q = node.command[node.qidx].copy()
            assert np.all(np.abs(q - previous) <= node.joint_speed_limits / node.cfg['rate_hz'] + 1e-4)
            if k < 15:  # 加速段；工作空间边界停车允许立即减速
                assert np.max(np.abs(node.joint_velocity - previous_velocity)) <= node.cfg['max_joint_accel'] / node.cfg['rate_hz'] + 1e-3
            previous_velocity = node.joint_velocity.copy()
            travel = node.filtered - node.origin
            assert abs(travel[1]) <= node.cfg['workspace_dy'] + 1e-9
            assert -node.cfg['workspace_z_down'] <= travel[2] <= node.cfg['workspace_z_up']
            actual = node.fk(node.command) - node.origin
            assert abs(actual[1]) <= node.cfg['workspace_dy'] + 1e-9
            assert -node.cfg['workspace_z_down'] <= actual[2] <= node.cfg['workspace_z_up']
            for i, pub in enumerate(node.pubs):
                speed = pub.publish.call_args.args[0].vlim
                assert 0 < speed <= node.joint_speed_limits[i]
        joints()
        end = node.fk(node.measured_q())
        assert end[1] > start[1] + .03 and end[2] > start[2] + .015
        origin = node.origin.copy()
        palm(valid=.5)
        target = node.filtered.copy()
        publish_count = sum(pub.publish.call_count for pub in node.pubs)
        node.last_tick = time.monotonic() - 1 / node.cfg['rate_hz']
        node.tick()
        assert np.allclose(node.filtered, node.fk(node.measured_q()))
        assert sum(pub.publish.call_count for pub in node.pubs) == publish_count
        assert not np.any(node.tcp_velocity) and not np.any(node.joint_velocity)
        target = node.filtered.copy()
        palm(valid=0.)
        assert node.lost_since is not None
        node.lost_since -= node.cfg['chase_timeout'] + .1
        node.last_tick = time.monotonic() - 1 / node.cfg['rate_hz']
        node.tick()
        assert np.allclose(node.filtered, node.fk(node.measured_q()))
        for _ in range(5):
            joints()
            palm()
        assert np.array_equal(node.origin, origin)
        target = node.filtered.copy()
        node.palm_time = time.monotonic() - 1
        node.tick()
        assert np.allclose(node.filtered, node.fk(node.measured_q()))
        for _ in range(5):
            joints()
            palm()
        node.joint_velocity[:] = 1.0
        node.tcp_velocity[:] = 0.4
        node.on_reset(Bool(data=True))
        assert node.homing and not np.any(node.tcp_velocity) and not np.any(node.joint_velocity)
        node.joint_times = {}
        node.tick()
        assert node.anchor is None
        print('PASS: slow startup, direction, per-joint speed, acceleration, actual FK workspace, pause, chase timeout, stale palm/joints')
        check_delayed_feedback(node)
        check_delayed_feedback(node, reverse=True)
        check_delayed_feedback(node, reverse=True, rate=50.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

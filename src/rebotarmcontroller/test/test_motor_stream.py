import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rebotarmcontroller.motor_passthrough import MotorPassthrough


def make_stream(rate=100.):
    node = Mock()
    hardware = Mock(joint_names=['joint1', 'joint2'], has_gripper=False,
                    state_machine='IDLE', _safety_generation=0, _cmd_lock=threading.RLock())
    stream = MotorPassthrough(node, hardware, 'rebotarm', 'reject', command_rate=rate)
    return node, hardware, stream


def test_10ms_serial_timer_is_non_overlapping():
    node, _, stream = make_stream()
    args = node.create_timer.call_args
    assert args.args[0] == .01
    assert isinstance(args.kwargs['callback_group'], MutuallyExclusiveCallbackGroup)
    assert args.kwargs['callback_group'] is stream._pv_timer_group


def test_latest_target_is_batched_and_status_not_polled_per_send():
    node, hardware, stream = make_stream()
    callback = stream._make_joint_pos_vel_callback('joint1')
    for value in [0.1, 0.2, 0.3]:
        callback(SimpleNamespace(pos=value, vlim=1.0))
    stream._flush_pos_vel()
    hardware.send_arm_pos_vel_stream.assert_called_once_with({'joint1': .3}, {'joint1': 1.})
    node.publish_arm_status.assert_not_called()
    stream._flush_pos_vel()
    assert hardware.send_arm_pos_vel_stream.call_count == 1


def test_safety_generation_change_drops_pending_target():
    _, hardware, stream = make_stream()
    stream._make_joint_pos_vel_callback('joint1')(SimpleNamespace(pos=.3, vlim=1.))
    hardware._safety_generation += 1
    stream._flush_pos_vel()
    hardware.send_arm_pos_vel_stream.assert_not_called()


def test_state_transition_still_publishes_status_immediately():
    node, hardware, stream = make_stream()
    stream._make_joint_pos_vel_callback('joint1')(SimpleNamespace(pos=.3, vlim=1.))

    def send(*args):
        hardware.state_machine = 'LOWLEVEL_STREAMING'

    hardware.send_arm_pos_vel_stream.side_effect = send
    stream._flush_pos_vel()
    node.publish_arm_status.assert_called_once_with()


def test_safety_stop_blocks_pending_target():
    _, hardware, stream = make_stream()
    stream._make_joint_pos_vel_callback('joint1')(SimpleNamespace(pos=.3, vlim=1.))
    hardware.state_machine = 'SAFETY_STOPPED'
    stream._flush_pos_vel()
    hardware.send_arm_pos_vel_stream.assert_not_called()


@pytest.mark.parametrize('rate', [0., -1., 201., float('nan'), float('inf')])
def test_invalid_rate_is_rejected_before_creating_interfaces(rate):
    with pytest.raises(ValueError):
        make_stream(rate)

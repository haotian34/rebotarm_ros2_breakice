from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from rebotarm_msgs.msg import ArmStatus
from std_msgs.msg import String

from rebotarm_gesture_actions.trajectory_player_node import TrajectoryPlayerNode


def test_stop_cancels_track_and_release_does_not_resume():
    player = SimpleNamespace(
        state='TRACK', frames=[[1.0] * 6], frame_index=12, entry_from={'joint1': 1.0},
        _publish_status=Mock(), get_logger=Mock(return_value=Mock()),
    )
    TrajectoryPlayerNode._on_arm_status(player, ArmStatus(state_machine='SAFETY_STOPPED', enabled=True))
    assert player.state == 'IDLE'
    assert player.frames == []
    assert player.safety_blocked
    TrajectoryPlayerNode._on_play(player, String(data='heart_draw'))
    assert player.state == 'IDLE'
    TrajectoryPlayerNode._on_arm_status(player, ArmStatus(state_machine='IDLE', enabled=True))
    assert not player.safety_blocked
    assert player.state == 'IDLE'
    assert player.frames == []


@pytest.mark.parametrize('state', ['TRACK', 'ENTRY', 'RESET'])
@pytest.mark.parametrize('track', ['rock_push', 'heart_draw', 'paper_circle', 'paper_figure8'])
@pytest.mark.parametrize('joint,expected', [(f'joint{i}', 5.0 if i<=3 else 3.0) for i in range(1,7)])
def test_sdk_speed_limits_apply_to_each_joint(state, track, joint, expected):
    publisher = Mock()
    clock = Mock()
    clock.now.return_value.to_msg.return_value = Time()
    player = SimpleNamespace(
        state=state, track_name=track, vlim=5.0, rock_vlim=5.0, figure8_vlim=5.0,
        joint_pubs={joint: publisher}, current={}, get_clock=lambda: clock,
    )
    TrajectoryPlayerNode._command(player, joint, -0.6)
    assert publisher.publish.call_args.args[0].vlim == pytest.approx(expected)


def test_wrist_does_not_exceed_sdk_config_with_large_override():
    publisher, clock = Mock(), Mock()
    clock.now.return_value.to_msg.return_value = Time()
    player = SimpleNamespace(
        state='TRACK', track_name='rock_push', vlim=100.0, rock_vlim=100.0,
        joint_pubs={'joint5': publisher}, current={}, get_clock=lambda: clock,
    )
    TrajectoryPlayerNode._command(player, 'joint5', 0.2)
    assert publisher.publish.call_args.args[0].vlim == 3.0

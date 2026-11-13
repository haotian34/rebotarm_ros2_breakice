from types import SimpleNamespace
from unittest.mock import Mock, patch
from rebotarm_voice.voice_control_node import VoiceControlNode


def state(count=1):
    return SimpleNamespace(play=Mock(get_subscription_count=Mock(return_value=count)),
                           pending=False, player_state='IDLE', status_at=99.,
                           block_until=0., cfg={'status_timeout':3., 'cooldown_sec':2.})


def test_duplicate_player_reason_is_distinct_from_cooldown():
    node = state(2)
    assert not VoiceControlNode._ready(node)
    assert 'Duplicate' in VoiceControlNode._unavailable_reason(node)


def test_busy_completion_cooldown_and_heartbeat_allow_repeated_commands():
    node = state()
    with patch('rebotarm_voice.voice_control_node.time.monotonic', return_value=100.):
        VoiceControlNode._on_status(node, SimpleNamespace(data='BUSY:rock_push'))
        assert not VoiceControlNode._ready(node)
        assert 'still running' in VoiceControlNode._unavailable_reason(node)
    with patch('rebotarm_voice.voice_control_node.time.monotonic', return_value=101.):
        VoiceControlNode._on_status(node, SimpleNamespace(data='IDLE'))
        assert not VoiceControlNode._ready(node)
        assert 'Cooling' in VoiceControlNode._unavailable_reason(node)
    with patch('rebotarm_voice.voice_control_node.time.monotonic', return_value=104.):
        VoiceControlNode._on_status(node, SimpleNamespace(data='IDLE'))
        assert VoiceControlNode._ready(node)
        assert node.block_until == 103.

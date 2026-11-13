import pytest
from rebotarm_gesture_actions.player_lock import acquire_player_lock, PlayerAlreadyRunning


def test_duplicate_player_is_blocked_and_lock_released_on_close(tmp_path):
    owner = acquire_player_lock('rebotarm', 173, tmp_path)
    try:
        with pytest.raises(PlayerAlreadyRunning):
            acquire_player_lock('/rebotarm/', 173, tmp_path)
    finally:
        owner.close()
    acquire_player_lock('rebotarm', 173, tmp_path).close()


def test_other_namespace_or_domain_is_independent(tmp_path):
    owner = acquire_player_lock('rebotarm', 173, tmp_path)
    try:
        acquire_player_lock('another_arm', 173, tmp_path).close()
        acquire_player_lock('rebotarm', 174, tmp_path).close()
    finally:
        owner.close()

from unittest.mock import Mock, patch

from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import String
from rebotarm_voice.rviz_subtitles import RvizSubtitlesNode


def node():
    instance = object.__new__(RvizSubtitlesNode)
    instance.text, instance.status = '', ''
    instance.visible_until = 0.0
    instance.timeout = 8.0
    instance.width, instance.text_size = 640, 22.0
    instance.publisher = Mock()
    return instance


def test_startup_clears_old_overlay():
    instance = node()
    instance._publish()
    assert instance.publisher.publish.call_args.args[0].action == OverlayText.DELETE


def test_transcript_expires_without_replaying_old_words():
    instance = node()
    with patch('rebotarm_voice.rviz_subtitles.time.monotonic', return_value=10.0):
        instance._text(String(data='画一个圆'))
    assert '画一个圆' in instance.publisher.publish.call_args.args[0].text
    with patch('rebotarm_voice.rviz_subtitles.time.monotonic', return_value=18.1):
        instance._publish()
    assert instance.text == ''
    assert instance.publisher.publish.call_args.args[0].action == OverlayText.DELETE
    with patch('rebotarm_voice.rviz_subtitles.time.monotonic', return_value=19.0):
        instance._status(String(data='READY'))
    assert '画一个圆' not in instance.publisher.publish.call_args.args[0].text


def test_new_recording_replaces_last_transcript():
    instance = node()
    instance._text(String(data='旧指令'))
    instance._status(String(data='LISTENING'))
    assert instance.text == ''
    assert '正在录音' in instance.publisher.publish.call_args.args[0].text

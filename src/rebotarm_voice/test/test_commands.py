import pytest
from rebotarm_voice.commands import parse_command


@pytest.mark.parametrize('text,track', [
    ('机械臂，请画爱心。', 'heart_draw'), ('机械臂出拳', 'rock_push'),
    ('机械臂画８字！', 'paper_figure8'), ('机械臂，画圆', 'paper_circle'),
])
def test_allowed(text, track):
    assert parse_command(text) == track


@pytest.mark.parametrize('text', ['', '画爱心', '不要机械臂画爱心',
    '机械臂不要出拳', '机械臂出拳然后画圆', '机械臂画爱心谢谢',
    '机械臂停止', '电视里说机械臂出拳'])
def test_reject(text):
    assert parse_command(text) is None


def test_custom_wake_word():
    assert parse_command('小白请画心', '小白') == 'heart_draw'
    assert parse_command('画心', '') == 'heart_draw'

import pytest
from rebotarm_voice.commands import parse_command


@pytest.mark.parametrize('text,track', [
    ('请画爱心。', 'heart_draw'), ('出拳', 'rock_push'),
    ('画８字！', 'paper_figure8'), ('画圆', 'paper_circle'),
    ('画八字', 'paper_figure8'), ('画心', 'heart_draw'),
    ('画个圆', 'paper_circle'), ('向前出拳', 'rock_push'),
])
def test_allowed(text, track):
    assert parse_command(text) == track


@pytest.mark.parametrize('text', ['', '不要画爱心', '不要出拳',
    '出拳然后画圆', '画爱心谢谢', '停止', '电视里说出拳', '机械臂出拳'])
def test_reject(text):
    assert parse_command(text) is None


def test_custom_wake_word():
    assert parse_command('小白请画心', '小白') == 'heart_draw'
    assert parse_command('画心', '小白') is None
    assert parse_command('画心', '') == 'heart_draw'

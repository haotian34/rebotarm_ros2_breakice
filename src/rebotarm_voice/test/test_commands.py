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
    '出拳然后画圆', '停止', '电视里说出拳'])
def test_reject(text):
    assert parse_command(text) is None


def test_custom_wake_word():
    assert parse_command('小白请画心', '小白') == 'heart_draw'
    assert parse_command('画心', '小白') is None
    assert parse_command('画心', '') == 'heart_draw'


@pytest.mark.parametrize('text,track', [
    ('请帮我画一个爱心吧', 'heart_draw'),
    ('机械臂画个圆圈谢谢', 'paper_circle'),
    ('能不能帮我画一下', None),
    ('能不能帮我画个八字', 'paper_figure8'),
    ('来打个拳', 'rock_push'), ('帮我划个园', 'paper_circle'),
    ('画爱星', 'heart_draw'), ('画爱心谢谢', 'heart_draw'),
])
def test_natural_commands(text, track):
    assert parse_command(text) == track


@pytest.mark.parametrize('text', [
    '请不要画圆', '别帮我画心', '画心或者画圆', '画圆再画心',
    '你知道画圆吗', '如果画圆', '画圆圈圈圈', '花园', '画',
    '不能画圆', '不要出拳画圆', '画圆不好',
])
def test_fuzzy_rejections(text):
    assert parse_command(text) is None


def test_strict_mode():
    assert parse_command('请画圆', fuzzy=False) == 'paper_circle'
    assert parse_command('帮我画个圆圈', fuzzy=False) is None


@pytest.mark.parametrize('text,track', [
    ('畫一個圓', 'paper_circle'), ('畫一个圆', 'paper_circle'),
    ('請幫我畫一個愛心吧', 'heart_draw'), ('機械臂畫個八字', 'paper_figure8'),
    ('揮個拳', 'rock_push'), ('別畫圓', None), ('不要畫愛心', None),
    ('畫圓然後出拳', None), ('電視裡說畫圓', None),
])
def test_traditional_chinese(text, track):
    assert parse_command(text) == track

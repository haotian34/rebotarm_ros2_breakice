from types import SimpleNamespace
from unittest.mock import MagicMock
import sys
import numpy as np
import pytest
from rebotarm_voice.audio import LocalRecognizer


def test_local_load_and_transcription(monkeypatch):
    model = MagicMock()
    factory = MagicMock(return_value=model)
    monkeypatch.setitem(sys.modules, 'faster_whisper', SimpleNamespace(WhisperModel=factory))
    sd = SimpleNamespace(rec=MagicMock(return_value=np.full((16000, 1), 1000, dtype=np.int16)), wait=MagicMock())
    monkeypatch.setitem(sys.modules, 'sounddevice', sd)
    recognizer = LocalRecognizer()
    recognizer.load()
    assert factory.call_args.args == ('base',)
    assert factory.call_args.kwargs['local_files_only'] is True
    assert factory.call_args.kwargs['device'] == 'cpu'
    def transcribe(audio, **kw):
        assert audio.shape == (16000,)
        assert audio.dtype == np.float32
        assert np.allclose(audio, 1000 / 32768)
        assert kw['vad_filter'] is True
        assert kw['condition_on_previous_text'] is False
        assert kw['language'] == 'zh'
        # Real faster-whisper returns a lazy generator.
        return (SimpleNamespace(text=t) for t in ['机械臂', '画圆']), None
    model.transcribe.side_effect = transcribe
    for _ in range(2):
        assert recognizer.record_and_transcribe(1, 16000, None, .01, 'zh') == '机械臂画圆'
    factory.assert_called_once()
    sd.rec.return_value[:] = 0
    assert recognizer.record_and_transcribe(1, 16000, None, .01, 'zh') == ''
    assert model.transcribe.call_count == 2
    with pytest.raises(ValueError):
        recognizer.record_and_transcribe(1, 48000, None, .01, 'zh')


def test_unloaded_model(monkeypatch):
    monkeypatch.setitem(sys.modules, 'sounddevice', SimpleNamespace())
    with pytest.raises(RuntimeError):
        LocalRecognizer().record_and_transcribe(1, 16000, None, .01, 'zh')

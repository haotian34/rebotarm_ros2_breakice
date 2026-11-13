from types import SimpleNamespace
from unittest.mock import MagicMock
import sys
import numpy as np
import pytest
from rebotarm_voice.audio import LocalRecognizer
from rebotarm_voice.audio import record_until_silence


def test_local_load_and_transcription(monkeypatch):
    model = MagicMock()
    factory = MagicMock(return_value=model)
    monkeypatch.setitem(sys.modules, 'faster_whisper', SimpleNamespace(WhisperModel=factory))
    sd = SimpleNamespace(rec=MagicMock(return_value=np.full((16000, 1), 1000, dtype=np.int16)), wait=MagicMock())
    monkeypatch.setitem(sys.modules, 'sounddevice', sd)
    recognizer = LocalRecognizer()
    recognizer.load()
    assert factory.call_args.args == ('small',)
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
        return (SimpleNamespace(text=t, avg_logprob=-0.2, no_speech_prob=0.1)
                for t in ['画', '圆']), None
    model.transcribe.side_effect = transcribe
    for _ in range(2):
        assert recognizer.record_and_transcribe(1, 16000, None, .01, 'zh') == '画圆'
    factory.assert_called_once()
    sd.rec.return_value[:] = 0
    result = recognizer.record_and_transcribe(1, 16000, None, .01, 'zh')
    assert result == ''
    assert result.rejection_reason.startswith('quiet audio:')
    assert model.transcribe.call_count == 2
    with pytest.raises(ValueError):
        recognizer.record_and_transcribe(1, 48000, None, .01, 'zh')


def test_unloaded_model(monkeypatch):
    monkeypatch.setitem(sys.modules, 'sounddevice', SimpleNamespace())
    with pytest.raises(RuntimeError):
        LocalRecognizer().record_and_transcribe(1, 16000, None, .01, 'zh')


@pytest.mark.parametrize('logprob,no_speech', [(-1.2, 0.1), (-0.2, 0.9)])
def test_reject_whole_low_confidence_utterance(monkeypatch, logprob, no_speech):
    monkeypatch.setitem(sys.modules, 'sounddevice', SimpleNamespace(
        rec=lambda *a, **kw: np.full((16000, 1), 1000, dtype=np.int16), wait=lambda: None))
    recognizer = LocalRecognizer()
    recognizer.model = MagicMock()
    recognizer.model.transcribe.return_value = (iter([
        SimpleNamespace(text='不要', avg_logprob=logprob, no_speech_prob=no_speech),
        SimpleNamespace(text='画圆', avg_logprob=-0.2, no_speech_prob=0.1),
    ]), None)
    result = recognizer.record_and_transcribe(1, 16000, None, .01, 'zh')
    assert result == '不要画圆'
    assert result.rejection_reason.startswith('low confidence:')


def fake_stream(levels, overflow=False):
    class Stream:
        reads = 0
        closed = False
        def __enter__(self):
            return self
        def __exit__(self, *args):
            self.closed = True
        def read(self, frames):
            level = levels[min(self.reads, len(levels) - 1)]
            self.reads += 1
            return np.full((frames, 1), level, dtype=np.int16), overflow
    stream = Stream()
    return stream, SimpleNamespace(InputStream=MagicMock(return_value=stream))


def test_endpoint_waits_for_speech_then_stops_after_silence():
    stream, sd = fake_stream([0]*20 + [1000]*4 + [0]*8)
    result = record_until_silence(sd, 8., 16000, None, .01, .8, .25)
    assert result.shape == (int(3.2*16000), 1)
    assert stream.closed


def test_endpoint_does_not_cut_a_short_pause_between_words():
    stream, sd = fake_stream([1000]*4 + [0]*4 + [1000]*4 + [0]*8)
    result = record_until_silence(sd, 8., 16000, None, .01, .8, .25)
    assert result.shape == (2*16000, 1)


@pytest.mark.parametrize('levels', [[0], [1000], [1000]+[0]*79])
def test_silence_continuous_speech_or_short_noise_obeys_cap(levels):
    stream, sd = fake_stream(levels)
    result = record_until_silence(sd, 8., 16000, None, .01, .8, .25)
    assert result.shape == (8*16000, 1)
    assert stream.closed


def test_audio_overflow_is_rejected_and_stream_closed():
    stream, sd = fake_stream([1000], overflow=True)
    with pytest.raises(RuntimeError, match='overflow'):
        record_until_silence(sd, 8., 16000, None, .01, .8, .25)
    assert stream.closed


def test_early_endpoint_flows_into_confidence_checked_transcription(monkeypatch):
    stream, sd = fake_stream([1000]*4 + [0]*8)
    monkeypatch.setitem(sys.modules, 'sounddevice', sd)
    recognizer = LocalRecognizer()
    recognizer.model = MagicMock()
    recognizer.model.transcribe.return_value = (iter([
        SimpleNamespace(text='打死', avg_logprob=-.2, no_speech_prob=.1)]), None)
    assert recognizer.record_and_transcribe(8.,16000,None,.01,'zh',silence_seconds=.8) == '打死'
    assert len(recognizer.model.transcribe.call_args.args[0]) == int(1.2*16000)
    assert stream.closed

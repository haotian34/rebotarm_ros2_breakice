"""Local Whisper inference. Runtime model loading is strictly offline."""


class Transcription(str):
    """Keep display text even when recognition is unsuitable for motion."""
    def __new__(cls, text, rejection_reason=''):
        result = super().__new__(cls, text)
        result.rejection_reason = rejection_reason
        return result


class LocalRecognizer:
    def __init__(self, model_size="small", model_device="cpu", compute_type="int8",
                 model_cache="", cpu_threads=4, min_avg_logprob=-0.8,
                 max_no_speech_prob=0.6):
        self.min_avg_logprob = min_avg_logprob
        self.max_no_speech_prob = max_no_speech_prob
        self.options = dict(device=model_device, compute_type=compute_type,
                            download_root=model_cache or None,
                            cpu_threads=cpu_threads, local_files_only=True)
        self.model_size = model_size
        self.model = None

    def load(self):
        from faster_whisper import WhisperModel
        self.model = WhisperModel(self.model_size, **self.options)

    def record_and_transcribe(self, seconds, sample_rate, device, min_rms, language):
        import numpy as np
        import sounddevice as sd

        if self.model is None:
            raise RuntimeError("Local Whisper model is not loaded")
        if sample_rate != 16000:
            raise ValueError("Local Whisper waveform input requires 16000 Hz")
        audio = sd.rec(int(seconds * sample_rate), samplerate=sample_rate,
                       channels=1, dtype="int16", device=device)
        sd.wait()
        waveform = audio.reshape(-1).astype(np.float32) / 32768.0
        rms = float(np.sqrt(np.mean(waveform ** 2)))
        if rms < min_rms:
            return Transcription('', f'quiet audio: rms={rms:.4f}, min_rms={min_rms:.4f}')
        segments, _ = self.model.transcribe(
            waveform, language=language, beam_size=5, temperature=0,
            condition_on_previous_text=False, vad_filter=True,
        )
        segments = list(segments)
        text = "".join(segment.text for segment in segments).strip()
        if not text:
            return Transcription('', f'no speech detected after VAD; rms={rms:.4f}')
        # Reject the entire utterance: dropping one segment could remove a
        # negation or one of two actions and change the customer's intent.
        if any(segment.avg_logprob < self.min_avg_logprob
               or segment.no_speech_prob > self.max_no_speech_prob
               for segment in segments):
            return Transcription(text, 'low confidence: '
                                 f'avg_logprob={min(s.avg_logprob for s in segments):.3f}, '
                                 f'no_speech_prob={max(s.no_speech_prob for s in segments):.3f}')
        return Transcription(text)


def download_model():
    """Explicit one-time download; runtime itself never downloads model files."""
    import argparse
    from faster_whisper.utils import download_model as fetch
    parser = argparse.ArgumentParser(description="Download a local Whisper model (no API key)")
    parser.add_argument("--model", default="small")
    parser.add_argument("--cache-dir", default=None)
    args = parser.parse_args()
    print(fetch(args.model, cache_dir=args.cache_dir))


if __name__ == "__main__":
    download_model()

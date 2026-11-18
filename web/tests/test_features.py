"""Feature lifecycle regressions independent of camera/ROS devices."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import Mock

spec=importlib.util.spec_from_file_location('studio_features',Path(__file__).resolve().parents[1]/'feature_session.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FeatureLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.features=module.FeatureSession(None)
        self.features.closed.set()
        self.features.thread.join(2)

    def tearDown(self):
        self.features.close()

    def test_continued_uploads_do_not_hide_failed_inference_forever(self):
        features=self.features
        features.bridge=SimpleNamespace(lock=threading.RLock(),state=lambda:{})
        features.vision='camera'
        features.states['camera']={'status':'starting','error':'','started_at':time.monotonic()-26}
        features.frame_at=time.monotonic()
        features._poll()
        self.assertIsNone(features.vision)
        self.assertEqual(features.states['camera']['status'],'error')

    def test_disabling_voice_during_inference_prevents_late_motion(self):
        features=self.features
        features.states['voice']={'status':'running','error':''}
        entered,release=threading.Event(),threading.Event()
        def transcribe(*args,**kwargs):
            entered.set()
            self.assertTrue(release.wait(3))
            return [SimpleNamespace(text='画爱心',avg_logprob=0,no_speech_prob=0)],None
        features.recognizer=SimpleNamespace(model=SimpleNamespace(transcribe=transcribe))
        features.transcript_pub=SimpleNamespace(publish=Mock())
        features._bridge=lambda:SimpleNamespace(String=lambda **kwargs:kwargs)
        features.text=Mock()
        errors=[]
        def recognize():
            try:
                features.audio(b'\x00\x20'*16000)
            except ValueError as exc:
                errors.append(str(exc))
        worker=threading.Thread(target=recognize)
        worker.start()
        self.assertTrue(entered.wait(3))
        features.stop('voice')
        release.set();worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertTrue(errors and '关闭' in errors[0])
        features.text.assert_not_called()
        self.assertFalse(features.voice_busy)

    def test_read_only_controller_refuses_feature_before_process_start(self):
        features=self.features
        def refuse(command):
            raise ValueError('Read only')
        features._bridge=lambda:SimpleNamespace(_check=refuse)
        features._spawn=Mock()
        for name in ('gesture','palm','voice','agent'):
            with self.assertRaisesRegex(ValueError,'Read only'):
                features.start(name)
            self.assertEqual(features.states[name]['status'],'stopped')
        features._spawn.assert_not_called()


if __name__=='__main__':
    unittest.main()

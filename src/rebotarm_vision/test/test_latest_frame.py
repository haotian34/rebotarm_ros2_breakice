from threading import Event

from rebotarm_vision.latest_frame import LatestFrame, OpenPalmGate


def test_camera_queue_keeps_only_latest_frame():
    drained, released = Event(), Event()

    class Capture:
        count = 0

        def read(self):
            self.count += 1
            if self.count <= 3:
                return True, self.count
            drained.set()
            released.wait(0.01)
            return False, None

        def release(self):
            released.set()

    capture = Capture()
    frames = LatestFrame(capture, lambda: capture.count)
    try:
        assert drained.wait(1.0)
        assert frames.take() == (3, 3)
        assert frames.take() is None
    finally:
        frames.close()
    assert released.is_set()


def test_uncertain_frame_does_not_stop_recent_open_palm():
    gate = OpenPalmGate(.1)
    assert gate.update('none', 1, 1.0) == .5
    assert gate.update('paper', 1, 1.1) == 1.0
    assert gate.update('none', 1, 1.15) == 1.0
    assert gate.update('none', 1, 1.25) == .5
    assert gate.update('none', 1, 1.26) == .5


def test_fist_and_two_hands_pause_immediately():
    gate = OpenPalmGate(.1)
    for raw, count, validity in [('rock', 1, .5), ('heart', 2, .5), ('none', 0, 0.)]:
        assert gate.update('paper', 1, 1.) == 1.
        assert gate.update(raw, count, 1.01) == validity
        assert gate.update('none', 1, 1.02) == .5

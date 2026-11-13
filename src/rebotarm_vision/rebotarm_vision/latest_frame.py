"""Keep only the newest camera frame while inference runs."""
from threading import Event, Lock, Thread


class LatestFrame:
    def __init__(self, capture, stamp):
        self.capture, self.stamp = capture, stamp
        self._stop, self._lock = Event(), Lock()
        self._latest = None
        self._thread = Thread(target=self._read, name='camera-latest-frame', daemon=True)
        self._thread.start()

    def _read(self):
        try:
            while not self._stop.is_set():
                ok, frame = self.capture.read()
                if not ok:
                    self._stop.wait(0.01)
                    continue
                item = frame, self.stamp()
                with self._lock:
                    self._latest = item
        finally:
            self.capture.release()

    def take(self):
        with self._lock:
            item, self._latest = self._latest, None
        return item

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


class OpenPalmGate:
    """Bridge brief uncertain classifications; fists/two hands pause immediately."""
    def __init__(self, grace_sec=0.0):
        self.grace_sec = grace_sec
        self.last_open = None

    def update(self, raw, count, now):
        if count == 1 and raw == 'paper':
            self.last_open = now
            return 1.0
        if (count == 1 and raw == 'none' and self.last_open is not None
                and now - self.last_open <= self.grace_sec):
            return 1.0
        self.last_open = None
        return 0.5 if count else 0.0

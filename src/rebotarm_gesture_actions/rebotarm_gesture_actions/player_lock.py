"""Host-local player ownership, independent of ROS discovery timing."""
import fcntl
import hashlib
import os
from pathlib import Path
import tempfile


class PlayerAlreadyRunning(RuntimeError):
    pass


def acquire_player_lock(namespace, domain=None, directory=None):
    domain = os.environ.get('ROS_DOMAIN_ID', '0') if domain is None else str(domain)
    key = hashlib.sha256(f'{domain}:{namespace.strip("/")}'.encode()).hexdigest()[:20]
    path = Path(directory or tempfile.gettempdir()) / f'rebotarm-player-{os.getuid()}-{key}.lock'
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    handle = os.fdopen(fd, 'a')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise PlayerAlreadyRunning(f'Player already running for /{namespace.strip("/")}; reusing existing player') from None
    # Do not unlink: replacing the inode would allow competing owners.
    return handle

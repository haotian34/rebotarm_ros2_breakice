"""Choose a Linux colour camera without opening a video stream."""

import fcntl
import os
from pathlib import Path
import struct


def realsense_rgb(path):
    interface = Path('/sys/class/video4linux') / path.resolve().name / 'device/interface'
    try:
        return 'rgb' in interface.read_text().lower()
    except OSError:
        return False


def colour_capture(path):
    """Inspect V4L2 formats, excluding depth, infrared and metadata nodes."""
    try:
        fd = os.open(str(path), os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        for index in range(64):
            data = bytearray(64)
            struct.pack_into("II", data, 0, index, 1)  # VIDEO_CAPTURE
            try:
                fcntl.ioctl(fd, 0xC0405602, data)  # VIDIOC_ENUM_FMT
            except OSError:
                break
            if bytes(data[44:48]) in {b"YUYV", b"UYVY", b"MJPG", b"JPEG", b"RGB3", b"BGR3"}:
                return True
        return False
    finally:
        os.close(fd)


def select_camera(mode, device="", index=0):
    if device:
        return int(device) if device.isdecimal() else device
    if mode == "manual":
        return index
    if mode not in {"real", "sim"}:
        raise ValueError("camera_mode must be manual, real or sim")
    candidates = {}
    for path in sorted(Path("/dev/v4l/by-id").glob("*")):
        is_realsense = "realsense" in path.name.lower()
        if is_realsense != (mode == "real"):
            continue
        # Some RealSense depth interfaces advertise UYVY too. Formats alone
        # cannot identify the RGB sensor; inspect the USB interface label.
        if is_realsense and not realsense_rgb(path):
            continue
        resolved = path.resolve()
        if resolved not in candidates and colour_capture(path):
            candidates[resolved] = str(path)
    if len(candidates) != 1:
        raise RuntimeError(
            f"camera_mode={mode}: expected one colour camera, found {len(candidates)}: "
            f"{list(candidates.values())}. Set camera_device:=/dev/v4l/by-id/<device> "
            "or REBOTARM_REAL_CAMERA / REBOTARM_SIM_CAMERA explicitly. "
            "Auto real mode selects RealSense; sim mode selects other webcams."
        )
    return next(iter(candidates.values()))

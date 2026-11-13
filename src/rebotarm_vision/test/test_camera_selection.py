from pathlib import Path
import unittest
from unittest.mock import patch

from rebotarm_vision.camera_selection import select_camera


class CameraSelectionTest(unittest.TestCase):
    def test_explicit_source_preserves_legacy_index(self):
        self.assertEqual(select_camera("manual", index=3), 3)
        self.assertEqual(select_camera("real", "4"), 4)
        self.assertEqual(select_camera("sim", "/dev/custom"), "/dev/custom")

    def test_modes_choose_different_colour_devices(self):
        paths = [Path("/dev/v4l/by-id/RealSense-depth"),
                 Path("/dev/v4l/by-id/RealSense-rgb"),
                 Path("/dev/v4l/by-id/laptop-rgb")]
        with patch.object(Path, "glob", return_value=paths), patch(
            "rebotarm_vision.camera_selection.colour_capture",
            return_value=True,
        ), patch(
            "rebotarm_vision.camera_selection.realsense_rgb",
            side_effect=lambda path: 'rgb' in path.name,
        ):
            self.assertEqual(select_camera("real"), str(paths[1]))
            self.assertEqual(select_camera("sim"), str(paths[2]))

    def test_missing_real_camera_does_not_fall_back_to_laptop(self):
        with patch.object(Path, "glob", return_value=[Path("/dev/laptop")]):
            with self.assertRaises(RuntimeError):
                select_camera("real")

    def test_multiple_webcams_require_explicit_selection(self):
        with patch.object(Path, "glob", return_value=[Path("/dev/a"), Path("/dev/b")]), patch(
            "rebotarm_vision.camera_selection.colour_capture", return_value=True
        ):
            with self.assertRaises(RuntimeError):
                select_camera("sim")

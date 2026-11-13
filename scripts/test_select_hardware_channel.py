from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from select_hardware_channel import select_channel


class ChannelSelectionTest(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = Path(self.temp.name) / 'hardware.yaml'
        self.config.write_text('default_model: dm\nmodels:\n  dm:\n    overrides:\n      channel: /dev/ttyACM0\n  rs:\n    overrides:\n      channel: can0\n')

    def test_explicit_argument_overrides_environment(self):
        self.assertEqual(select_channel(self.config, ['channel:=/dev/custom'],
                                        {'REBOTARM_CHANNEL': '/dev/other'}), '/dev/custom')

    def test_environment_persists_selected_device(self):
        self.assertEqual(select_channel(self.config, [], {'REBOTARM_CHANNEL': '/dev/stable'}), '/dev/stable')

    def test_rs_keeps_socketcan(self):
        self.assertEqual(select_channel(self.config, ['model:=rs'], {}), 'can0')

    def test_reenumerated_serial_uses_stable_path(self):
        target = Path(self.temp.name) / 'ttyACM1'
        stable = Path(self.temp.name) / 'usb-controller'
        stable.symlink_to(target)
        with patch.object(Path, 'glob', return_value=[stable]), patch.object(Path, 'is_char_device', return_value=True):
            self.assertEqual(select_channel(self.config, [], {}), str(stable))

    def test_multiple_adapters_are_not_guessed(self):
        paths = [Path(self.temp.name) / 'ttyACM1', Path(self.temp.name) / 'ttyACM2']
        with patch.object(Path, 'glob', return_value=paths), patch.object(Path, 'is_char_device', return_value=True):
            with self.assertRaisesRegex(ValueError, 'Multiple USB serial devices'):
                select_channel(self.config, [], {})

    def test_no_device_fails_before_launch(self):
        with patch.object(Path, 'glob', return_value=[]):
            with self.assertRaises(ValueError):
                select_channel(self.config, [], {})

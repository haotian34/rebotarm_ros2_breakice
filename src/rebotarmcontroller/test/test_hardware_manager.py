import math
import unittest

from rebotarmcontroller.hardware_manager import HardwareManager


class GripperPositionLimitTest(unittest.TestCase):
    @staticmethod
    def _hardware(open_position, close_position):
        hardware = object.__new__(HardwareManager)
        hardware.gripper_open_position = float(open_position)
        hardware.gripper_close_position = float(close_position)
        hardware._configure_gripper_position_limits()
        return hardware

    def test_dm_limits_use_configured_range(self):
        hardware = self._hardware(open_position=-5.0, close_position=0.0)

        for position in (-5.0, -1.0, 0.0):
            with self.subTest(position=position):
                self.assertEqual(hardware.validate_gripper_position(position), position)

        for position in (-5.1, 2.5, math.nan, math.inf):
            with self.subTest(position=position):
                with self.assertRaises(ValueError):
                    hardware.validate_gripper_position(position)

    def test_rs_limits_use_configured_range(self):
        hardware = self._hardware(open_position=5.0, close_position=0.0)

        for position in (0.0, 1.0, 5.0):
            with self.subTest(position=position):
                self.assertEqual(hardware.validate_gripper_position(position), position)

        for position in (5.1, -1.0, math.nan, math.inf):
            with self.subTest(position=position):
                with self.assertRaises(ValueError):
                    hardware.validate_gripper_position(position)

    def test_reversed_open_and_close_ordering_is_supported(self):
        hardware = self._hardware(open_position=0.0, close_position=3.0)

        self.assertEqual(hardware.validate_gripper_position(1.5), 1.5)

    def test_invalid_limit_configuration_is_rejected(self):
        cases = (
            (-5.0, -5.0),
            (math.nan, 0.0),
            (5.0, math.inf),
        )
        for open_position, close_position in cases:
            with self.subTest(
                open_position=open_position,
                close_position=close_position,
            ):
                with self.assertRaises(ValueError):
                    self._hardware(open_position, close_position)


if __name__ == "__main__":
    unittest.main()


class SafetyLatchTest(unittest.TestCase):
    def setUp(self):
        import threading
        from types import SimpleNamespace
        import numpy as np
        self.hardware = object.__new__(HardwareManager)
        h = self.hardware
        h._cmd_lock = threading.RLock()
        h._safety_latched = True
        h._safety_generation = 2
        h._state_machine = 'SAFETY_HOMING'
        h._endpos_ctrl = SimpleNamespace(_q_target=np.array([0.5]), _qd_target=np.zeros(1))
        h.get_joint_positions = lambda **kwargs: np.array([0.2])

    def test_motion_commands_rejected_after_stop(self):
        h = self.hardware
        for command, args in ((h.enable, ()), (h.set_joint_position_target, ([1.0],)),
                              (h.send_joint_pos_vel_cmd, ('joint1', 1.0, 0.5)),
                              (h.set_gripper_target, (0.0,)),
                              (h.start_gravity_compensation, ())):
            with self.subTest(command=command.__name__):
                with self.assertRaises(RuntimeError):
                    command(*args)

    def test_late_action_cleanup_cannot_clear_latch_or_target(self):
        h = self.hardware
        h.set_state_machine('IDLE')
        h.hold_current_position()
        self.assertEqual(h.state_machine, 'SAFETY_HOMING')
        self.assertEqual(h._endpos_ctrl._q_target[0], 0.5)

    def test_old_home_worker_cannot_write_after_second_stop(self):
        import numpy as np
        self.hardware._safety_home_loop(1, np.array([1.0]), 3.0)
        self.assertEqual(self.hardware._endpos_ctrl._q_target[0], 0.5)

    def test_unsafe_velocity_rejected(self):
        for velocity in (0, -1, 0.2, math.inf, math.nan):
            with self.assertRaises(ValueError):
                self.hardware.stop_and_home(velocity)

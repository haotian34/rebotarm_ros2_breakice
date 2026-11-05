"""Safety behavior on the actual fake-driver node without moving hardware."""
import pytest
import rclpy
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from rebotarmcontroller.fake_rs_driver import FakeRsDriver


@pytest.fixture
def driver():
    rclpy.init()
    node = FakeRsDriver()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stop_holds_arm_and_gripper_and_rejects_commands(driver):
    driver.positions[0] = 0.8
    driver.targets[0] = 1.2
    driver.gripper_position = 1.0
    driver.gripper_target = 4.0
    result = driver._safety_stop(None, Trigger.Response())
    assert result.success
    driver._set_joint_target(0, 2.0)
    driver._set_gripper_target(3.0)
    assert driver.targets == driver.positions
    assert driver.gripper_target == 1.0
    for callback in (driver._enable, driver._safe_home,
                     driver._start_gravity_compensation, driver._stop_gravity_compensation):
        assert not callback(None, Trigger.Response()).success
    assert driver.state_machine == 'SAFETY_STOPPED'


def test_home_speed_and_second_stop(driver):
    driver.positions[0] = 1.0
    driver.targets[0] = 2.0
    result = driver._stop_and_home(None, Trigger.Response())
    assert result.success
    assert driver.state_machine == 'SAFETY_HOMING'
    driver.last_time = driver.get_clock().now() - Duration(seconds=1)
    driver._tick()
    assert 0.89 < driver.positions[0] < 0.91
    assert abs(driver.velocities[0]) <= 0.100001
    result = driver._stop_and_home(None, Trigger.Response())
    assert not result.success
    assert driver.state_machine == 'SAFETY_STOPPED'
    assert driver.targets == driver.positions


def test_home_finishes_with_latch_active(driver):
    driver.positions[0] = 0.05
    driver._stop_and_home(None, Trigger.Response())
    driver.last_time = driver.get_clock().now() - Duration(seconds=1)
    driver._tick()
    assert driver.positions[0] == 0.0
    assert driver.state_machine == 'SAFETY_STOPPED'
    assert driver.safety_latched


def test_disabled_driver_never_starts_home(driver):
    driver._disable(None, Trigger.Response())
    driver.positions[0] = 0.8
    assert not driver._stop_and_home(None, Trigger.Response()).success
    driver.last_time = driver.get_clock().now() - Duration(seconds=1)
    driver._tick()
    assert driver.positions[0] == 0.8


def test_last_home_steps_keep_speed_limit(driver):
    driver.positions[0] = 0.0019
    driver._stop_and_home(None, Trigger.Response())
    driver.last_time = driver.get_clock().now() - Duration(seconds=0.001)
    driver._tick()
    assert driver.state_machine == 'SAFETY_HOMING'
    assert abs(driver.velocities[0]) <= 0.100001


def test_duplicate_namespace_is_rejected_and_can_restart(driver):
    with pytest.raises(RuntimeError, match="Refusing duplicate fake driver"):
        FakeRsDriver()


def test_gripper_stop_survives_continuous_stream_and_ticks(driver):
    driver.gripper_position = 1.2
    driver.gripper_target = 5.0
    driver._safety_stop(None, Trigger.Response())
    for target in [0.0, 5.0] * 20:
        driver._set_gripper_target(target)
        driver.last_time = driver.get_clock().now() - Duration(seconds=0.01)
        driver._tick()
        assert driver.gripper_position == 1.2
        assert driver.gripper_velocity == 0.0
    assert driver.safety_latched


def test_instance_guard_releases_on_destroy():
    rclpy.init()
    first = FakeRsDriver()
    first.destroy_node()
    second = None
    try:
        second = FakeRsDriver()
    finally:
        if second is not None:
            second.destroy_node()
        rclpy.shutdown()

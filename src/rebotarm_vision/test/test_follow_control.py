import pytest
from rebotarm_vision.follow_control import cartesian_lag_scale, joint_lag_scale


def test_cartesian_corrective_and_tangent_motion_not_slowed_by_old_gap():
    assert cartesian_lag_scale([0, .05, 0], [0, -.2, 0], .06) == 1.
    assert cartesian_lag_scale([0, .05, 0], [0, 0, .2], .06) == 1.
    assert 0 < cartesian_lag_scale([0, .05, 0], [0, .2, 0], .06) < 1.


@pytest.mark.parametrize('gap,velocity', [([.09, 0], [-.2, 0]), ([.09, 0], [0, .2])])
def test_joint_reversal_and_tangent_motion_not_slowed(gap, velocity):
    assert joint_lag_scale(gap, velocity) == 1.


def test_outward_motion_still_slows_before_hard_stop():
    assert joint_lag_scale([.04, 0], [.3, .2]) == 1.
    assert 0 < joint_lag_scale([.08, 0], [.3, .2]) < 1.
    assert joint_lag_scale([.11, 0], [.3, .2]) == 0.


def test_large_stationary_joint_error_does_not_slow_another_axis():
    assert joint_lag_scale([.11, .01], [0, .2]) == 1.

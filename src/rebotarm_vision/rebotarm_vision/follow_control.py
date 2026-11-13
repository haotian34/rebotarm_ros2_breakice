"""Feedback-lag scaling that leaves corrective and tangential motion responsive."""
import numpy as np


def cartesian_lag_scale(gap, velocity, max_lead):
    speed = float(np.linalg.norm(velocity))
    # Only displacement along the requested motion contributes to further lag.
    outward_gap = max(0.0, float(np.dot(gap, velocity)) / max(speed, 1e-9))
    return float(np.clip((max_lead - outward_gap) / (max_lead * .65), 0., 1.))


def joint_lag_scale(gap, velocity):
    outward = np.asarray(gap) * np.asarray(velocity) > 0.0
    if not np.any(outward):
        return 1.0
    lag = float(np.max(np.abs(np.asarray(gap)[outward])))
    return float(np.clip((.105 - lag) / .065, 0., 1.))

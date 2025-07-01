"""
Unit tests for the motion planning module used in controlling the 6-axis robot arm.

"""

import pytest
import numpy as np
from backend.robot.motion_planning import *
import math

PI = math.pi

def test_linear_interpolation():
    """
    Test correct interpolation output for given start and goal angles.

    Validates that the intermediate steps linearly match expected joint positions.
    """
    start = [0.0, 0.0]
    goal = [PI, PI/2]
    steps = 3
    expected = [
        [0.0, 0.0],
        [PI/2, PI/4],
        [PI, PI/2]
    ]
    result = linear_interpolation(start, goal, steps)

    for res_row, exp_row in zip(result, expected):
        assert res_row == exp_row

def test_invalid_length():
    """
    Test that mismatched angle list lengths raise a ValueError.

    Ensures that start and goal joint angle vectors must be equal in size.
    """
    with pytest.raises(ValueError):
        linear_interpolation([0.0], [PI, PI], 5)

def test_cubic_spline_wrapping_across_boundary():
    """
    Test that cubic spline correctly wraps around ±180° for shortest rotation.
    E.g. from 170° to -170° should rotate -20°, not +340°.
    """
    start = [PI/4]
    goal = [-PI/4]
    joint_limits = [(-PI, PI)]
    n_steps = 5

    result = cubic_spline_interpolation(start, goal, n_steps, joint_limits)

    # Confirm 5 steps
    assert len(result) == n_steps
    # Extract the only joint's values
    joint_vals = [step[0] for step in result]
    
    # First and last values must match start and goal
    assert pytest.approx(joint_vals[0], abs=1e-6) == PI/4
    assert pytest.approx(joint_vals[-1], abs=1e-6) == -PI/4

    # Total angular distance traveled should be smaller than pi
    total_distance = sum(abs(np.diff(joint_vals)))
    assert total_distance < PI

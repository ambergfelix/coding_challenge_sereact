"""
Unit tests for the motion planning module used in controlling the 6-axis robot arm.

"""

import pytest
import numpy as np
from backend.robot.motion_planning import *

def test_linear_interpolation():
    """
    Test correct interpolation output for given start and goal angles.

    Validates that the intermediate steps linearly match expected joint positions.
    """
    start = [0.0, 0.0]
    goal = [10.0, 20.0]
    steps = 3
    expected = [
        [0.0, 0.0],
        [5.0, 10.0],
        [10.0, 20.0]
    ]
    result = linear_interpolation(start, goal, steps)

    for res_row, exp_row in zip(result, expected):
        assert res_row == pytest.approx(exp_row, abs=1e-6)

def test_invalid_length():
    """
    Test that mismatched angle list lengths raise a ValueError.

    Ensures that start and goal joint angle vectors must be equal in size.
    """
    with pytest.raises(ValueError):
        linear_interpolation([0.0], [1.0, 2.0], 5)
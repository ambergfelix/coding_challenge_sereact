"""
Unit tests for the motion planning module used in controlling the 6-axis robot arm.

"""

import pytest
import numpy as np
from robot.motion_planning import *
import utils.utils as util
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
    Test that cubic spline interpolation correctly wraps angles to follow the shortest rotational path.

    Ensures that the actual traveled angular distance is smaller or equal to pi.
    """
    start_angles = [np.radians(-170)]
    goal_angles = [np.radians(170)]
    joint_limits = [(-PI, PI)]
    n_steps = 5

    trajectory = cubic_spline_interpolation(
        start_angles=start_angles,
        goal_angles=goal_angles,
        n_steps=n_steps,
        joint_limits=joint_limits
    )

    unwrapped = np.unwrap(trajectory)
    total_path_length = np.sum(np.abs(np.diff(unwrapped)))


    assert total_path_length < PI, f"Expected shortest path, but took longer: {np.degrees(total_path_length)} degrees"


def test_cubic_spline_with_viapoints():
    """
    Test cubic spline interpolation with two custom via points, ensuring they are present in the trajectory.
    """
    start = [0.0]
    goal = [PI / 2]
    joint_limits = [(-PI, PI)]
    via_points = [[PI / 6, PI / 3]]  # Two via points for the single joint
    n_steps = 10

    trajectory = cubic_spline_interpolation(start, goal, n_steps, joint_limits, via_points)

    # Confirm trajectory has correct number of steps
    assert len(trajectory) == n_steps

    # Extract only joint's values from trajectory
    joint_vals = [step[0] for step in trajectory]

    # Check that via points are close to any step value
    matches = [
        any(pytest.approx(val, abs=1e-3) == PI / 6 for val in joint_vals),
        any(pytest.approx(val, abs=1e-3) == PI / 3 for val in joint_vals)
    ]

    assert all(matches), "Via points not found in trajectory"


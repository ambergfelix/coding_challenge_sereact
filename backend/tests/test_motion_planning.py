import pytest
import numpy as np
from robot.motion_planning import *

def test_linear_interpolation():
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
    with pytest.raises(ValueError):
        linear_interpolation([0.0], [1.0, 2.0], 5)
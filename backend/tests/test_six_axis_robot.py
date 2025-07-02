"""
Unit tests for the 6-axis robot module.

"""

import pytest
from robot.joint import Joint
from robot.six_axis_robot import SixAxisRobot
import utils.utils as util
import math
from pathlib import Path
import json

PI = math.pi

@pytest.fixture
def six_axis_robot():
    # Load config
    config_path = Path("/robot-model/ur5/ur5_config.json")
    with open(config_path) as f:
        config = json.load(f)
    robot = util.robot_from_config(config)
    yield robot


def test_joint_within_limit():
    """
    Test that a joint accepts and stores an angle within its defined limits.
    """
    limit = PI/4
    joint = Joint("Test", -limit, limit)
    joint.set_angle(PI/5)
    assert joint.get_angle() == pytest.approx(PI/5, abs=1e-6)

def test_joint_above_limit():
    """
    Test that a joint input exceeding the maximum is clamped to the upper limit.
    """
    limit = PI/4
    joint = Joint("Test", -limit, limit)
    joint.set_angle(PI/2)
    assert joint.get_angle() == pytest.approx(limit, abs=1e-6)

def test_joint_below_limit():
    """
    Test that a joint input below the minimum is clamped to the lower limit.
    """
    limit = PI/4
    joint = Joint("Test", -limit, limit)
    joint.set_angle(-PI/2)
    assert joint.get_angle() == pytest.approx(-limit, abs=1e-6)

def test_set_robot_joint_angles(six_axis_robot):
    """
    Test that setting joint angles updates all robot joints correctly.
    """
    angles = [PI/4, -PI/2, PI/4, -PI/2, PI/6, PI]
    six_axis_robot.set_joint_angles(angles)
    actual_angles = [j.get_angle() for j in six_axis_robot.joints]
    for actual, expected in zip(actual_angles, angles):
        assert actual == pytest.approx(expected, abs=1e-6)


def test_six_axis_robot_invalid_input_lenght(six_axis_robot):
    """
    Test that an invalid number of joint angles raises a ValueError.
    """
    angles = [PI/2, -PI/2, PI/2, -PI/2]
    with pytest.raises(ValueError):
        six_axis_robot.set_joint_angles(angles)

def test_pose_for_zero_angles(six_axis_robot):
    """
    Test the end-effector pose when the robot is in a zero-degree configuration.
    Verifies that the computed pose matches the known expected position.
    """
    zero_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    six_axis_robot.set_joint_angles(zero_angles)
    pose = six_axis_robot.get_pose().t_3_1
    x, y, z = pose.flatten()

    expected_x = -0.81725
    expected_y = -0.19145
    expected_z = -0.005491

    assert x == pytest.approx(expected_x, abs=1e-6)
    assert y == pytest.approx(expected_y, abs=1e-6)
    assert z == pytest.approx(expected_z, abs=1e-6)
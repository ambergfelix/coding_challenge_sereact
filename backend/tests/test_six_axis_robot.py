"""
Unit tests for the 6-axis robot module.

"""

import pytest
from backend.robot.joint import Joint
from backend.robot.six_axis_robot import SixAxisRobot
import math

PI = math.pi

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

def test_set_robot_joint_angles():
    """
    Test that setting joint angles updates all robot joints correctly.
    """
    robot = SixAxisRobot()
    angles = [PI/4, -PI/2, PI/4, -PI/2, PI/6, PI]
    robot.set_joint_angles(angles)
    actual_angles = [j.get_angle() for j in robot.joints]
    for actual, expected in zip(actual_angles, angles):
        assert actual == pytest.approx(expected, abs=1e-6)


def test_six_axis_robot_invalid_input_lenght():
    """
    Test that an invalid number of joint angles raises a ValueError.
    """
    robot = SixAxisRobot()
    angles = [PI/2, -PI/2, PI/2, -PI/2]
    with pytest.raises(ValueError):
        robot.set_joint_angles(angles)

def test_pose_for_zero_angles():
    """
    Test the end-effector pose when the robot is in a zero-degree configuration.
    Verifies that the computed pose matches the known expected position.
    """
    robot = SixAxisRobot()
    zero_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot.set_joint_angles(zero_angles)
    pose = robot.pose.t_3_1
    x, y, z = pose.flatten()

    expected_x = -0.81725
    expected_y = -0.19145
    expected_z = -0.005491

    assert x == pytest.approx(expected_x, abs=1e-6)
    assert y == pytest.approx(expected_y, abs=1e-6)
    assert z == pytest.approx(expected_z, abs=1e-6)
"""
Unit tests for the 6-axis robot module.

"""

import pytest
from robot.joint import Joint
from robot.six_axis_robot import Six_axis_robot

def test_joint_within_limit():
    """
    Test that a joint accepts and stores an angle within its defined limits.
    """
    joint = Joint("Test", -45, 45)
    joint.set_angle(25)
    assert joint.get_angle() == 25

def test_joint_above_limit():
    """
    Test that a joint input exceeding the maximum is clamped to the upper limit.
    """
    limit = 45
    joint = Joint("Test", -limit, limit)
    joint.set_angle(90)
    assert joint.get_angle() == limit

def test_joint_below_limit():
    """
    Test that a joint input below the minimum is clamped to the lower limit.
    """
    limit = 45
    joint = Joint("Test", -limit, limit)
    joint.set_angle(-90)
    assert joint.get_angle() == -limit

def test_joint_repr():
    """
    Test that the string representation of a joint is formatted correctly.
    """
    joint = Joint("Test", -180, 180)
    joint.set_angle(90)
    assert repr(joint) == "<Joint Test: 90.0 deg>"

def test_set_robot_joint_angles():
    """
    Test that setting joint angles updates all robot joints correctly.
    """
    robot = Six_axis_robot()
    angles = [20, -30, 20, -30, 20, 30]
    robot.set_joint_angles(angles)
    actual_angles = [j.get_angle() for j in robot.joints]
    assert actual_angles == angles

def test_six_axis_robot_invalid_input_lenght():
    """
    Test that an invalid number of joint angles raises a ValueError.
    """
    robot = Six_axis_robot()
    angles = [90, -90, 90, -90]
    with pytest.raises(ValueError):
        robot.set_joint_angles(angles)

def test_pose_for_zero_angles():
    """
    Test the end-effector pose when the robot is in a zero-degree configuration.
    Verifies that the computed pose matches the known expected position.
    """
    robot = Six_axis_robot()
    zero_angles = [0, 0, 0, 0, 0, 0]
    robot.set_joint_angles(zero_angles)
    pose = robot.pose.t_3_1
    x, y, z = pose.flatten()

    expected_x = -0.81725
    expected_y = -0.19145
    expected_z = -0.005491

    assert x == pytest.approx(expected_x, abs=1e-6)
    assert y == pytest.approx(expected_y, abs=1e-6)
    assert z == pytest.approx(expected_z, abs=1e-6)

def test_six_axis_repr_output():
    """
    Test that the __repr__ output of the robot includes the correct joint names and angles.
    """
    robot = Six_axis_robot()
    test_angles = [10, -20, 30, -40, 50, -60]
    robot.set_joint_angles(test_angles)
    repr_output = repr(robot)
    lines = repr_output.strip().split("\n")

    expected_names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"]
    assert len(lines) == 6
    for name, angle, line in zip(expected_names, test_angles, lines):
        assert name in line
        assert f"{angle:.1f} deg" in line
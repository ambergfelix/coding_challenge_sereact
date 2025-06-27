import pytest
from robot.joint import Joint
from robot.six_axis_robot import Six_axis_robot


def test_joint_within_limit():
    joint = Joint("Test", -45, 45)
    joint.set_angle(25)
    assert joint.get_angle() == 25

def test_joint_above_limit():
    limit = 45
    joint = Joint("Test", -limit, limit)
    joint.set_angle(90)
    assert joint.get_angle() == limit

def test_joint_below_limit():
    limit = 45
    joint = Joint("Test", -limit, limit)
    joint.set_angle(-90)
    assert joint.get_angle() == -limit

def test_joint_repr():
    joint = Joint("Test", -180, 180)
    joint.set_angle(90)
    assert repr(joint) == "<Joint Test: 90.0 deg>"

# Test Six_axis_robot

def set_robot_joint_angles():
    robot = Six_axis_robot()
    angles = [20, -30, 20, -30, 20]
    robot.set_angles(angles)
    actual_angles = robot.get_joint_states()
    assert actual_angles == angles

def test_six_axis_robot_invalid_input_lenght():
    robot = Six_axis_robot()
    angles = [90, -90, 90, -90]
    with pytest.raises(ValueError):
        robot.set_joint_angles(angles)

def test_six_axis_repr_output():
    robot = Six_axis_robot()
    # Set a known configuration
    test_angles = [10, -20, 30, -40, 50, -60]
    robot.set_joint_angles(test_angles)

    # Get the string representation
    repr_output = repr(robot)
    lines = repr_output.strip().split("\n")

    # Check that each line contains the corresponding joint name and angle
    expected_names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"]
    assert len(lines) == 6
    for name, angle, line in zip(expected_names, test_angles, lines):
        assert name in line
        assert f"{angle} deg" in line


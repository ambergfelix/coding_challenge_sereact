"""
Unit tests for the collision checking module
"""

import pytest
import pybullet as pyblt
from robot.collision_checker import CollisionChecker
from pathlib import Path
import math

PI = math.pi

@pytest.fixture
def collision_checker():
    """
    Fixture for setting up the CollisionChecker with the UR5 robot model.
    Ensures PyBullet disconnects after tests run.
    """
    path_urdf = str(Path("robot-model/ur5/ur5.urdf"))
    checker = CollisionChecker(path_urdf)
    yield checker
    pyblt.disconnect()

def test_no_self_collision_initial(collision_checker):
    """
    Test that the robot does not self-collide in its initial position.
    """
    assert not collision_checker.check_self_collision(), "Robot should not self-collide initially."

def test_self_collision(collision_checker):
    """
    Test that setting rotating elbow to 90 degree causes self-collision.
    """
    angles = [0, 0, PI, 0, 0, 0]
    collision_checker.set_angles(angles)
    assert collision_checker.check_self_collision(), "Robot should self-collide at shoulder angle = pi"

def test_floor_collision(collision_checker):
    """
    Test that shoulder rotation to +45 degree causes the robot to collide with the floor.
    """
    angles = [0, PI/2, 0, 0, 0, 0]
    collision_checker.set_angles(angles)
    assert collision_checker.check_floor_collision(), "Robot should collide with floor."

def test_no_floor_collision(collision_checker):
    """
    Test that shoulder rotation to -45 degree causes the robot to not collide with the floor.
    """
    angles = [0, -PI/2, 0, 0, 0, 0]
    collision_checker.set_angles(angles)
    assert not collision_checker.check_floor_collision(), "Robot should not collide with floor."

def test_add_sphere_obstacle(collision_checker):
    """
    Test that adding a spherical obstacle increases the obstacle count.
    """
    collision_checker.add_obstacle((0, 0, 1), 'sphere', 0.1)
    assert len(collision_checker.obstacles) == 1

def test_add_box_obstacle(collision_checker):
    """
    Test that adding a box obstacle increases the obstacle count.
    """
    collision_checker.add_obstacle((1, 1, 1), 'box', 0.2)
    assert len(collision_checker.obstacles) == 1

def test_obstacle_collision(collision_checker):
    """
    Test that a collision with an obstacle is detected correctly.
    """
    collision_checker.add_obstacle((0, 0, 0), 'sphere', 0.5)
    assert collision_checker.check_obstacle_collision()

def test_remove_obstacle(collision_checker):
    """
    Test that removing an obstacle decreases the obstacle count.
    """
    collision_checker.add_obstacle((0, 0, 1), 'box', 0.2)
    initial = len(collision_checker.obstacles)
    collision_checker.remove_obstacle(0)
    assert len(collision_checker.obstacles) == initial - 1

@pytest.mark.xfail(reason="Feature not fully implemented yet")
def test_set_angles(collision_checker):
    """
    Test that setting joint angles correctly updates joint positions.
    """
    angle = PI/4
    movable_joint_indices = [
        i for i in range(pyblt.getNumJoints(collision_checker.robot_model))
        if pyblt.getJointInfo(collision_checker.robot_model, i)[2] != pyblt.JOINT_FIXED
    ]
    angles = [angle] * len(movable_joint_indices)
    collision_checker.set_angles(angles)
    states = collision_checker.get_angles()
    for idx, state in zip(movable_joint_indices, states):
        assert state == pytest.approx(angle, abs=1e-3), f"Joint {idx} should be {angle} rad but is '{state} rad'"
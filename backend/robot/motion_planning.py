"""
Motion planning utilities for controlling the 6-axis robotic arm.

This module provides basic functions for generating and executing joint-space
trajectories based on linear interpolation.
"""

import numpy as np
from typing import List
from .six_axis_robot import SixAxisRobot
import time

def linear_interpolation(start_angles: List[float], goal_angles: List[float], n_steps: int) -> List[List[float]]:
    """
    Generates a list of intermediate joint angle configurations using linear interpolation.

    Parameters
    ----------
    start_angles : List[float]
        Starting joint angles in degrees for each of the robot's 6 axes
    goal_angles : List[float]
        Target joint angles in degrees for each of the robot's 6 axes
    n_steps : int
        Number of interpolation steps between start and goal

    Returns
    -------
    List[List[float]]
        A list of joint angle lists representing the trajectory

    Raises
    ------
    ValueError
        If the number of angles in the start and goal vectors do not match
    """
    if len(start_angles) != len(goal_angles):
        raise ValueError("Length of start angles and goal angles not matching.")
    steps = np.linspace(start_angles, goal_angles, num=n_steps)
    return steps.tolist()

def execute_movement(robot: SixAxisRobot, path: List[List[float]], delay: float = 1/30):
    """
    Executes a given motion path by sequentially applying joint angles to the robot.

    Parameters
    ----------
    robot : Six_axis_robot
        Instance of the robot that supports joint angle updates
    path : List[List[float]]
        A time-ordered list of joint angle vectors representing the motion sequence
    delay : float, optional
        Pause duration in seconds between each step (default is ~33ms for 30 FPS playback)
    """
    for step in path:
        robot.set_joint_angles(step)
        time.sleep(delay)
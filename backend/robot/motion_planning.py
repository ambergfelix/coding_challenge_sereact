"""
Motion planning utilities for controlling the 6-axis robotic arm.

This module provides basic functions for generating and executing joint-space
trajectories based on linear interpolation.
"""

import numpy as np
from typing import List, Tuple
from backend.robot.six_axis_robot import SixAxisRobot
import time
from scipy.interpolate import CubicSpline


def cubic_spline_interpolation(start_angles: List[float], goal_angles: List[float], n_steps: int, joint_limits: List[Tuple[float, float]]) -> List[List[float]]:
    var_indep = [0, 0.5, 1]
    steps = []
    for start, goal, limits in zip(start_angles, goal_angles, joint_limits):
        low, high = limits

        diff = goal - start
        
        # Adjust difference to the shortest rotation angle:
        # If diff > 180°, rotating forward is longer than going backward,
        # so subtract 360° to rotate counterclockwise (negative direction).
        # If diff < -180°, rotating backward is longer than going forward,
        # so add 360° to rotate clockwise (positive direction).
        if diff > 180:
            diff_shortest = diff - 360
        elif diff < -180:
            diff_shortest = diff + 360
        else:
            diff_shortest = diff

        viapoint = start + (diff_shortest / 2)
        end = start + diff_shortest
        var_dep = [start, viapoint, end]

        spline = CubicSpline(var_indep, var_dep)
        t_values = np.linspace(0, 1, n_steps)
        angle_values = spline(t_values)
        
        if all(low <= v <= high for v in angle_values):
            steps.append(angle_values)
        else: 
            # Rotation in other direction using longer path
            viapoint = start + (diff / 2)
            end = start + diff
            var_dep = [start, viapoint, end]

            spline = CubicSpline(var_indep, var_dep)
            angle_values = spline(t_values)
            steps.append(angle_values)


    
    # Transpose to get list of joint angle configurations at each step
    steps_t = [list(step) for step in zip(*steps)]
    return steps_t
    

def linear_interpolation(start_angles: List[float], goal_angles: List[float], n_steps: int) -> List[List[float]]:
    """
    Generates a list of intermediate joint angle configurations using linear interpolation
    with shortest path rotation for each joint in [-180,180] degrees.

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
    
    steps = []
    
    for start, goal in zip(start_angles, goal_angles):
        diff = goal - start
        
        # Adjust difference to the shortest rotation angle:
        # If diff > 180°, rotating forward is longer than going backward,
        # so subtract 360° to rotate counterclockwise (negative direction).
        # If diff < -180°, rotating backward is longer than going forward,
        # so add 360° to rotate clockwise (positive direction).
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360

        
        interpolation = np.linspace(start, start + diff, n_steps)
        steps.append(interpolation)

    
    # Transpose to get list of joint angle configurations at each step
    steps_t = [list(step) for step in zip(*steps)]
    return steps_t

def execute_movement(robot: SixAxisRobot, path: List[List[float]], delay: float = 1/60):
    """
    Executes a given motion path by sequentially applying joint angles to the robot.

    Parameters
    ----------
    robot : Six_axis_robot
        Instance of the robot that supports joint angle updates
    path : List[List[float]]
        A time-ordered list of joint angle vectors representing the motion sequence
    delay : float, optional
        Pause duration in seconds between each step (default is ~17ms for 60 FPS playback)
    """
    for step in path:
        robot.set_joint_angles(step)
        time.sleep(delay)
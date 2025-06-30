"""
Motion planning utilities for controlling the 6-axis robotic arm.

This module provides basic functions for generating and executing joint-space
trajectories based on linear interpolation.
"""

import numpy as np
from typing import List, Tuple, Optional
from backend.robot.six_axis_robot import SixAxisRobot
from backend.robot.collision_checker import CollisionChecker
import backend.utils.utils as util
import time
from scipy.interpolate import CubicSpline
import math

PI = math.pi


def cubic_spline_interpolation(start_angles: List[float], goal_angles: List[float], n_steps: int, joint_limits: List[Tuple[float, float]], via_points: Optional[List[List[float]]] = None) -> List[List[float]]:
    """
    Generates a joint-space trajectory between two angle configurations using cubic spline interpolation
    with one or more points in between. The function ensures joint limits are respected and selects the
    shortest feasible rotational path when possible.

    If the shortest path would violate joint limits, the longer alternative path is used instead.

    Parameters
    ----------
    start_angles : List[float]
        Initial joint angles in radians for each of the robot's 6 axes
    goal_angles : List[float]
        Target joint angles in radians for each of the robot's 6 axes
    n_steps : int
        Number of interpolation steps between start and goal configurations
    joint_limits : List[Tuple[float, float]]
        List of (min, max) angle bounds for each joint in radians
    via_points : Optional[List[List[float]]] = None
        Optional list of via points for interpolation, if none are passed, the middle of the path is set as the only viapoint

    Returns
    -------
    List[List[float]]
        A list of joint angle configurations representing the interpolated trajectory
        where each sublist corresponds to one time step and contains 6 joint values

    Notes
    -----
    - The goal angles are clamped to their respective joint limits before interpolation.
    - The function attempts to use the shortest path in angular space (i.e., minimal rotation),
      and only falls back to the longer path if required to stay within limits.
    """

    steps = []

    if via_points is None or len(via_points) == 0:
        via_points = [[] for x in range(len(start_angles))]

    for start, goal, limits, points in zip(start_angles, goal_angles, joint_limits, via_points):
        low, high = limits
        viapoint_given = False if points==[] else True

        # Clamp goal angle to joint's rotational limit if it is outside the limits
        if goal > high:
            goal = high
        elif goal < low:
            goal = low
        
        diff = goal - start
        
        # Adjust difference to the shortest rotation angle:
        # If diff > pi (180°), rotating forward is longer than going backward,
        # so subtract 2pi (360°) to rotate counterclockwise (negative direction).
        # If diff < -pi (-180°), rotating backward is longer than going forward,
        # so add 2pi (360°) to rotate clockwise (positive direction).
        if diff > PI:
            diff_shortest = diff - 2*PI
        elif diff < -PI:
            diff_shortest = diff + 2*PI
        else:
            diff_shortest = diff

        end = start + diff_shortest

        

        if not viapoint_given:
            var_indep = [0, 0.5, 1]
            points = [start + (diff_shortest / 2)]
        else:
            # Calculate independent variables to achieve equal angle speed
            all_points = [start] + points + [goal]
            distances = [abs(all_points[i+1] - all_points[i]) for i in range(len(all_points)-1)]
            sum_distance = sum(distances)
            distances_norm = np.array(distances) / sum_distance
            var_indep = np.insert(np.cumsum(distances_norm), 0, 0)

        var_dep = [start] + points + [end]

        spline = CubicSpline(var_indep, var_dep)
        t_values = np.linspace(0, 1, n_steps)
        angle_values = spline(t_values)
        
        if all(low <= util.normalize_angle(v) <= high for v in angle_values):
            steps.append(angle_values)
        else: 
            # Rotation in other direction using longer path
            if not viapoint_given:
                viapoint = start + (diff / 2)
                end = start + diff
                var_dep = [start, viapoint, end]
            end = start + diff
            var_dep = [start] + points + [end]

            spline = CubicSpline(var_indep, var_dep)
            angle_values = spline(t_values)
            steps.append(angle_values)

    # Transpose to get list of joint angle configurations at each step
    steps_t = [list(step) for step in zip(*steps)]
    return steps_t

def linear_interpolation_collision_avoidance(start_angles: List[float], goal_angles: List[float], n_steps: int) -> List[List[float]]:
    via_configs = []
    perturb_range = 0.2
    collision_configs = []
    
    path = linear_interpolation(start_angles, goal_angles, n_steps)
    # Simualte path execution
    # robot_sim = SixAxisRobot()
    # robot_sim.set_joint_angles(start_angles)
    path_urdf = ""
    collision_check = CollisionChecker(path_urdf)
    collision_check.set_angles(start_angles)

    for step in path:
        collision_check.set_angles(step)
        # robot_sim.set_joint_angles(step)
        if collision_check.check_self_collision():
            collision_configs.append(step)
        if collision_check.check_environment_collision():
            collision_configs.append(step)
    
    for config in collision_configs:
        via_configs.append(config + np.random.uniform(-perturb_range, perturb_range, size=step.shape))

    # Verify if new trajectory is collision free
    path_new = linear_interpolation()
        



            



    

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
        # If diff > pi (180°), rotating forward is longer than going backward,
        # so subtract 2pi (360°) to rotate counterclockwise (negative direction).
        # If diff < -pi (-180°), rotating backward is longer than going forward,
        # so add 2pi (360°) to rotate clockwise (positive direction).
        if diff > PI:
            diff -= 2*PI
        elif diff < -PI:
            diff += 2*PI

        
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
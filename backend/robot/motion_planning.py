"""
Motion planning utilities for controlling the 6-axis robotic arm.

This module provides basic functions for generating and executing joint-space
trajectories based on linear interpolation.
"""

import numpy as np
from typing import List, Tuple, Optional
from robot.six_axis_robot import SixAxisRobot
from robot.collision_checker import CollisionChecker
import utils.utils as util
import time
from scipy.interpolate import CubicSpline
import math
from pathlib import Path
import warnings

PI = math.pi

def cubic_spline_interpolation_collision_avoidance(start_angles: List[float], goal_angles: List[float], n_steps: int, joint_limits: List[Tuple[float, float]]) -> List[List[float]]:
    """
    Generates a collision-free joint path using cubic spline interpolation with perturbation-based avoidance.

    This function attempts to create a smooth trajectory from a start to goal joint configuration
    while avoiding collisions (self, floor, and environment). It verifies that the goal configuration is collision free first, then
    repeatedly simulates the path and introduces randomized viapoints around collision configurations until either a 
    valid trajectory is found or a maximum number of iterations is reached.

    Parameters
    ----------
    start_angles : List[float]
        The starting joint configuration (in radians).
    goal_angles : List[float]
        The target joint configuration (in radians).
    n_steps : int
        Number of interpolated steps to generate between start and goal.
    joint_limits : List[Tuple[float, float]]
        A list of tuples specifying the (min, max) limits for each joint.

    Returns
    -------
    List[List[float]]
        A list of joint angle configurations representing the full interpolated path.
        If no valid path is found after max iterations, returns the start configuration only.

    Notes
    -----
    - If the goal configuration is already in collision, the function immediately returns the start configuration.
    - If collisions are encountered during path execution, the function introduces randomized
      perturbations to create new viapoints around collision point in configuration space, refining the trajectory to avoid obstacles.
    """

    
    
    # Additional viapoints that include random pertubations
    via_configs = []
    # increase pertubation range with increasing steps!
    perturb_range = 0.2
    # Number of iterations until it is terminated
    max_iter = 50
    n_iter = 0
    
    
    path_urdf = str(Path("/robot-model/ur5/ur5.urdf"))
    collision_check = CollisionChecker(path_urdf)

    # Check if goal configuration is in collision
    collision_check.set_angles(goal_angles)
    if collision_check.check_any_collision():
        warnings.warn("Collision detected at goal configuration; reverting to start.")
        path = [start_angles]
        return path
    

    # Create initial path without additional viapoints
    path = cubic_spline_interpolation(start_angles, goal_angles, n_steps, joint_limits)

    # Simualte path execution
    while n_iter < max_iter:
        perturb_range *= n_iter
        n_iter +=1
        collision = False
        via_configs = []
        collision_check.set_angles(start_angles)

        for step in path:
            collision_check.set_angles(step)
            if collision_check.check_any_collision():
                via_configs.append(step + np.random.uniform(-perturb_range, perturb_range, size=len(step)))
                collision = True
        if not collision:
            return path
        else:
            # If there is at least one collision create new path with pertubated viapoints
            path = cubic_spline_interpolation(start_angles, goal_angles, n_steps, joint_limits, via_configs)

    # If no valid path was found within the max iterations return start configuration
    path = [start_angles]
    return path

        


def cubic_spline_interpolation(start_angles: List[float], goal_angles: List[float], n_steps: int, joint_limits: List[Tuple[float, float]], via_points: Optional[List[List[float]]] = None) -> List[List[float]]:
    """
    Generates a joint-space trajectory using cubic spline interpolation between start and goal angles.

    Supports optional viapoints to guide the spline path through intermediate configurations.
    Ensures all joint angles remain within their specified limits and uses shortest rotational paths
    unless infeasible due to constraints.

    Parameters
    ----------
    start_angles : List[float]
        Initial joint angles in radians for each joint.
    goal_angles : List[float]
        Target joint angles in radians for each joint.
    n_steps : int
        Number of discrete points to generate along the trajectory.
    joint_limits : List[Tuple[float, float]]
        A list of (min, max) joint angle boundaries for each joint (in radians).
    via_points : Optional[List[List[float]]], optional
        A list of intermediate joint angles to pass through, provided as a list.
        Each element in the outer list corresponds to one joint and should contain a list of intermediate
        angles for that joint. If None or empty, a single midpoint is automatically inserted for each joint.

    Returns
    -------
    List[List[float]]
        A list of joint angle configurations, one for each step along the trajectory.
        Each configuration is a list of joint angles in radians.

    Raises
    ------
    ValueError
        If the trajectory violates joint limits and no valid alternative path can be found.

    Notes
    -----
    - For angular wrapping, shortest rotation is attempted via ±2π normalization.
    - If via_points are provided, they will override automatic midpoint insertion.
    """


    steps = []

    if via_points is None or len(via_points) == 0:
        via_points = [[] for _ in range(len(start_angles))]

    for start, goal, limits, points in zip(start_angles, goal_angles, joint_limits, via_points):
        low, high = limits
        viapoint_given = False if points==[] else True

        # Clamp goal angle to joint's rotational limit if it is outside the limits
        if goal > high:
            goal = high
        elif goal < low:
            goal = low
        
        diff = goal - start

        if not viapoint_given:
            var_indep = [0, 0.5, 1]
            var_dep = np.unwrap([start, goal])
            viapoint = var_dep[0] + (var_dep[1] - var_dep[0]) / 2
            var_dep = [var_dep[0], viapoint, var_dep[1]]
        else:
            # Calculate independent variables to achieve equal angle speed
            var_dep = np.unwrap([start] + points + [goal])
            distances = [abs(var_dep[i+1] - var_dep[i]) for i in range(len(var_dep)-1)]
            sum_distance = sum(distances)
            distances_norm = np.array(distances) / sum_distance
            var_indep = np.insert(np.cumsum(distances_norm), 0, 0)

        spline = CubicSpline(var_indep, var_dep)
        t_values = np.linspace(0, 1, n_steps)
        angle_values = spline(t_values)
        angle_values = [util.normalize_angle(v) for v in angle_values]

        
        if all(low < v < high for v in angle_values):
            steps.append(angle_values)
        else: 
            # Rotation in other direction using longer path
            if not viapoint_given:
                viapoint = start + (diff / 2)
                var_dep = np.unwrap([start, viapoint, goal])
            else:
                # Raise error if shortest path between points not feasible due to joint limits
                # ToDo: evaluate invalid sections of spline and rotate in those in other direction
                raise ValueError("No valid cubic spline path found between viapoints within joint limits.")

            spline = CubicSpline(var_indep, var_dep)
            angle_values = spline(t_values)
            angle_values = [util.normalize_angle(v) for v in angle_values]
            steps.append(angle_values)

    # Transpose to get list of joint angle configurations at each step
    steps_t = [list(step) for step in zip(*steps)]
    return steps_t
    

def linear_interpolation(start_angles: List[float], goal_angles: List[float], n_steps: int) -> List[List[float]]:
    """
    Generates a list of intermediate joint angle configurations using linear interpolation
    with shortest path rotation for each joint in [-pi,pi] radians.

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
import numpy as np
from typing import List
from robot.six_axis_robot import Six_axis_robot
from typing import List
import time

def linear_interpolation(start_angles: List[float], goal_angles: List[float], n_steps: int)-> List[List[float]]:
    if len(start_angles) != len(goal_angles):
        raise ValueError("Length of start angles and goal angles not matching.")
    steps = np.linspace(start_angles, goal_angles, num=n_steps)
    return steps.tolist()

def execute_movement(robot: Six_axis_robot, path: List[List[float]], delay : float = 1/30):
    for step in path:
        robot.set_joint_angles(step)
        time.sleep(delay)
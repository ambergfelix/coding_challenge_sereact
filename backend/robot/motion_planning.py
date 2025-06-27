import numpy as np
import time
from typing import List

def linear_interpolation(start_angles: List[float], goal_angles: List[float], n_steps: int)-> List[List[float]]:
    if len(start_angles) != len(goal_angles):
        raise ValueError("Length of start angles and goal angles not matching.")
    steps = np.linspace(start_angles, goal_angles, num=n_steps)
    return steps.tolist()
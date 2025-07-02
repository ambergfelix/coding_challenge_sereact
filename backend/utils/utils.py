import math
from robot.six_axis_robot import SixAxisRobot
from robot.joint import Joint
import numpy as np
from typing import List, Dict, Union

PI = math.pi

def normalize_angle(angle: float) -> float:
    """
    Normalizes an angle in radians to the range [+pi, -pi]

    Parameters
    ----------
    angle : float
        The angle to be normalized

    Returns
    -------
    float
        The to the range normalized angle
    
    """
    return (angle + PI) % (2 * PI) - PI

def robot_from_config(config: Dict[str, Union[List[Joint], List[List[float]]]]) -> SixAxisRobot:
    """
    Creates a 6-axis robot configuration from a JSON configuration object.

    The input configuration should define the robot's joints and Denavit-Hartenberg (DH) parameters.
    
    Parameters:
    -----------
    config : dict
        A dictionary parsed from a JSON file with the following structure:
        
        {
            "joints": [
                {
                    "name": str,          # Name of the joint (e.g. "Base", "Shoulder")
                    "min_angle": float,   # Minimum rotation angle in radians
                    "max_angle": float    # Maximum rotation angle in radians
                },
                ...
            ],
            "dh_params": [
                [a, d, alpha, theta],   # DH parameters for each joint as a list of floats
                ...
            ]
        }

        - "joints" must contain exactly 6 joint definitions, one for each axis.
        - "dh_params" must contain 6 sets of four float values, corresponding to each joint's DH parameters.

    Returns:
    --------
    SixAxisRobot
        A robot model initialized with the specified joints and DH parameters.
    
    Raises:
    -------
    KeyError
        If the expected keys ("joints", "dh_params") are missing.
    ValueError
        If the number of joints or DH parameter sets is not equal to 6.

    Example:
    --------
    config = {
        "joints": [
            {"name": "Base", "min_angle": -3.14, "max_angle": 3.14},
            ...
        ],
        "dh_params": [
            [0.089159, 0.0, 1.57, 0.0],
            ...
        ]
    }
    robot = robot_from_config(config)
    """

    if "joints" not in config or "dh_params" not in config:
        raise KeyError("Configuration must contain 'joints' and 'dh_params' keys.")
    
    if len(config["joints"]) != 6:
        raise ValueError("Configuration must define exactly 6 joints.")
    
    if len(config["dh_params"]) != 6:
        raise ValueError("Configuration must define exactly 6 DH parameter sets.")

    joints = []
    for idx, joint_cfg in enumerate(config["joints"]):
        try:
            joint = Joint(
                name=joint_cfg["name"],
                min_angle=joint_cfg["min_angle"],
                max_angle=joint_cfg["max_angle"]
            )
        except KeyError as e:
            raise KeyError(f"Joint definition at index {idx} is missing a required field: {e}")
        joints.append(joint)

    dh_params = np.array(config["dh_params"])
    return SixAxisRobot(joints, dh_params)

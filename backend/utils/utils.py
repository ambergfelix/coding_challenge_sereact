import math
from backend.robot.six_axis_robot import SixAxisRobot
from backend.robot.joint import Joint
import numpy as np

PI = math.pi
def normalize_angle(angle):
    return (angle + PI) % (2 * PI) - PI

def robot_from_config(config):
    joints = []
    for joint_cfg in config["joints"]:
        joint = Joint(
            name=joint_cfg["name"],
            min_angle=joint_cfg["min_angle"],
            max_angle=joint_cfg["max_angle"]
        )
        joints.append(joint)
    dh_params = np.array(config["dh_params"])
    return SixAxisRobot(joints, dh_params)
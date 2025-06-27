from robot.joint import Joint
from visual_kinematics.RobotSerial import RobotSerial
from typing import List
import numpy as np

class Six_axis_robot:
    def __init__(self):
        self.joints = [
            Joint("Base", -180.0, 180.0),
            Joint("Shoulder", -90.0, 90),
            Joint("Elbow", -180, 180),
            Joint("Wrist1", -180, 180),
            Joint("Wrist2", -180, 180),
            Joint("Wrist3", -180, 180)
        ]

        # Define DH parameters for kinematics
        # DH parameters taken from Universal Robots UR5
        self.dh_parameters = np.array([
           # d [m]      a [m]       alpha [rad] theta [rad] 
            [0.089159,  0,          np.pi/2,    0],
            [0,         -0.425,     0,          0],
            [0,         -0.39225,   0,          0],
            [0.10915,   0,          np.pi/2,    0],
            [0.09465,   0,          -np.pi/2,   0],
            [0.0823,    0,          0,          0]
        ])

        self.model = RobotSerial(self.dh_parameters)
        self.pose = self.model.forward([0]*6)
    
    def set_joint_angles(self, angles_deg: List[float]):
        if len(angles_deg) != len(self.joints):
            raise ValueError("Angle list does not match joint count.")
        for joint, angle in zip(self.joints, angles_deg):
            joint.set_angle(angle)

        angles_rad = [np.radians(a) for a in angles_deg]
        self.pose = self.model.forward(angles_rad)

    def get_joint_states(self) -> List[str]:
        return [repr(joint) for joint in self.joints]

    def __repr__(self):
        return "\n".join(self.get_joint_states())




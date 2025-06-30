from backend.robot.joint import Joint
from visual_kinematics.RobotSerial import RobotSerial
from typing import List, Tuple
import numpy as np
import math

PI = math.pi

class SixAxisRobot:
    """
    Representation of a stationary 6-axis industrial robotic arm modeled after the Universal Robots UR5.

    This class initializes the joint structure and Denavit-Hartenberg parameters 
    for forward kinematics using the visual_kinematics library.
    
    Attributes
    ----------
    joints : list
        List containing each joint of the robot with its respective name and min/max rotation limits
    dh_parameters : np.ndarray
        Denavit-Hartenberg parameters used for computing forward kinematics
    model : RobotSerial
        Kinematic model of the robot using the visual_kinematics library
    pose : np.ndarray
        Current end-effector pose represented as a 4x4 transformation matrix
    """

    def __init__(self):
        """
        Constructor for a stationary 6-axis robotic arm.

        Initializes all revolute joints with predefined limits and sets up the DH parameters
        for calculating the forward kinematic pose.
        """
        self.joints = [
            Joint("Base",       -PI, PI),
            # Limit Shoulder to not hit the ground
            Joint("Shoulder",   -PI,  0.0),
            # Limit Elbow to not hit Shoulder
            Joint("Elbow",      -math.radians(160), math.radians(160)),
            Joint("Wrist1",     -PI, PI),
            Joint("Wrist2",     -PI, PI),
            Joint("Wrist3",     -PI, PI)
        ]

        # DH parameters taken from Universal Robots UR5
        self.dh_parameters = np.array([
          # d [m]       a [m]     alpha [rad] theta [rad]
            [0.089159,  0.0,          PI/2,         0.0],
            [0.0,         -0.425,     0.0,          0.0],
            [0.0,         -0.39225,   0.0,          0.0],
            [0.10915,   0.0,          PI/2,         0.0],
            [0.09465,   0.0,          -PI/2,        0.0],
            [0.0823,    0.0,          0.0,          0.0]
        ])

        self.model = RobotSerial(self.dh_parameters)
        self.pose = self.model.forward([0] * 6)

    def set_joint_angles(self, angles: List[float]):
        """
        Sets the joint angles of each joint in the robot, within the respective rotational limits.

        Parameters
        ----------
        angles : List[float]
            List of angles (in rad) to assign to each joint

        Raises
        ------
        ValueError
            If the number of provided angles does not match the number of joints
        """
        if len(angles) != len(self.joints):
            raise ValueError("Angle list does not match joint count.")
        for joint, angle in zip(self.joints, angles):
            joint.set_angle(angle)

        self.pose = self.model.forward(angles)
    
    def get_joint_angles(self) -> List[float]:
        """
        Returns a list of the current angles (in rad) for each joint.

        Returns
        -------
        List[float]
            The current angle of each joint in radians
        """
        return [joint.current_angle for joint in self.joints]
    
    def get_joint_limits(self) -> List[Tuple[float, float]]:
        """
        Returns a list of all rotational limits of each of the robot's joints

        Returns
        -------
        List[Tuple[float, float]]
            List containing a tuple of min and max roataion for each joint
        """
        print(self.joints[0].get_limits())
        limits = [joint.get_limits() for joint in self.joints]
        return limits

    def get_joint_states(self) -> List[str]:
        """
        Returns a list of string representations of each joint’s current state.

        Returns
        -------
        List[str]
            Formatted status strings showing name and angle of each joint
        """
        return [repr(joint) for joint in self.joints]

    def __repr__(self):
        """
        Returns a formatted multi-line string showing all joint states.

        Returns
        -------
        str
            Combined string of all joints' representations for inspection/logging
        """
        return "\n".join(self.get_joint_states())

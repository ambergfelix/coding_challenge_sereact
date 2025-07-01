import pybullet as pyblt
import pybullet_data
from typing import Tuple
from typing import List

import sys
import os


# Suppress stderr temporarily
class SuppressStderr:
    def __enter__(self):
        self.stderr = sys.stderr
        sys.stderr = open(os.devnull, 'w')

    def __exit__(self, *args):
        sys.stderr.close()
        sys.stderr = self.stderr


class CollisionChecker:
    def __init__(self, urdf_path: str):
        # Start pybullet physics engine without GUI
        pyblt.connect(pyblt.DIRECT)
        pyblt.setGravity(0, 0, 0)
        # Optionally disable joint motors
        pyblt.setAdditionalSearchPath(pybullet_data.getDataPath())
        #
        with SuppressStderr():
            self.robot_model = pyblt.loadURDF(urdf_path, useFixedBase=True, flags=pyblt.URDF_USE_SELF_COLLISION)
        #
        # self.robot_model = pyblt.loadURDF(urdf_path, useFixedBase=True, flags=pyblt.URDF_USE_SELF_COLLISION)
        for i in range(pyblt.getNumJoints(self.robot_model)):
            pyblt.setJointMotorControl2(self.robot_model, i, pyblt.VELOCITY_CONTROL, force=0)
        self.floor = pyblt.loadURDF("plane.urdf", useFixedBase=True)

        # List to store the added obstacles
        self.obstacles = []
    
    def check_self_collision(self) -> bool:
        # Perform collision check using joint_state
        body = self.robot_model
        # Check if robot touches itself
        contacts = pyblt.getContactPoints(bodyA=body, bodyB=body)
        return len(contacts) > 0

    def check_floor_collision(self) -> bool:
        # Check for floor collision
        contacts = pyblt.getContactPoints(bodyA=self.robot_model, bodyB=self.floor)
        for contact in contacts:
            link_index = contact[3]  # linkIndexA (robot's link in contact)
            
            # Skip the base if you want to ignore it (usually index -1 or 0)
            if link_index > 0:
                link_name = pyblt.getBodyInfo(self.robot_model)[0].decode("utf-8") \
                    if link_index == -1 else pyblt.getJointInfo(self.robot_model, link_index)[12].decode("utf-8")
                print(link_name)

        # Ignore base of robot that always touches the floor
        return len(contacts) > 1
    
    def check_obstacle_collision(self) -> bool:
        # Check for collision with obstacles
        for obstacle in self.obstacles:
            contacts = pyblt.getContactPoints(bodyA=self.robot_model, bodyB=obstacle)
            if len(contacts) > 0:
                return True
        return False

    def check_any_collision(self) -> bool:
        return self.check_self_collision() or self.check_floor_collision() or self.check_obstacle_collision()


    def set_angles(self, angles):
        # Get indices of movable joints (ignore fixed ones)
        movable_joints = [
            i for i in range(pyblt.getNumJoints(self.robot_model))
            if pyblt.getJointInfo(self.robot_model, i)[2] != pyblt.JOINT_FIXED
        ]

        if len(angles) != len(movable_joints):
            raise ValueError(f"Expected {len(movable_joints)} joint angles, but got {len(angles)}")

        # Set each angle for corresponding movable joint
        for joint_index, angle in zip(movable_joints, angles):
            pyblt.resetJointState(self.robot_model, jointIndex=joint_index, targetValue=angle)

        pyblt.stepSimulation()
    
    def get_angles(self) -> List[float]:
        # Get indices of movable joints (ignore fixed ones)
        movable_joints = [
            i for i in range(pyblt.getNumJoints(self.robot_model))
            if pyblt.getJointInfo(self.robot_model, i)[2] != pyblt.JOINT_FIXED
        ]

        angles = [pyblt.getJointState(self.robot_model, idx)[0] for idx in movable_joints]
        return angles



    def add_obstacle(self, pos: Tuple[float, float, float], type: str, size: float):
        if type == 'sphere':
            shape = pyblt.createCollisionShape(shapeType = pyblt.GEOM_SPHERE, radius = size)
            body = pyblt.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, basePosition=pos)
            self.obstacles.append(body)
            pyblt.stepSimulation()
        elif type == 'box':
            dims = (size, size, size)
            shape = pyblt.createCollisionShape(shapeType=pyblt.GEOM_BOX, halfExtents = dims)
            body = pyblt.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, basePosition=pos)
            self.obstacles.append(body)
            pyblt.stepSimulation()
        else:
            raise ValueError(f"Unknown obstacle type '{type}'. Expected 'sphere' or 'box'.")

    def remove_obstacle(self, idx: int):
        if not 0 <= idx < len(self.obstacles):
            raise ValueError(f"Invalid index: No obstacle with index '{idx}'")

        body_id = self.obstacles.pop(idx)
        pyblt.removeBody(body_id) 
            
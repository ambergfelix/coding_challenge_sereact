import pybullet as pyblt
import pybullet_data
from typing import Tuple
from typing import List

class CollisionChecker:
    """
    A utility class to perform collision detection for a URDF-based robot using the PyBullet physics engine.

    This class initializes a physics simulation without rendering (headless) and manages a robot model, 
    a planar floor, and custom static obstacles. It provides methods to check for collisions between 
    the robot and itself, the floor, or any added obstacle. It also supports manipulation of the robot's
    joint states and obstacle management within the scene.

    Parameters
    ----------
    urdf_path : str
        The file path to the robot's URDF model to be loaded into the simulation.

    Attributes
    ----------
    robot_model : int
        The unique body ID assigned to the loaded robot model by PyBullet.
    floor : int
        The body ID for the ground plane loaded into the simulation.
    obstacles : List[int]
        A list of PyBullet body IDs for obstacles added to the simulation.
    """

    def __init__(self, urdf_path: str):
        """
        Initializes the physics simulation environment, loads the robot and floor, and prepares for collision checks.

        Starts the PyBullet physics engine in headless (DIRECT) mode without gravity. Loads the robot model
        specified by the given URDF path with self-collision detection enabled, and sets all joints to passive mode.
        Also loads a static floor and initializes an empty list to track user-defined obstacles.

        Parameters
        ----------
        urdf_path : str
            Path to the robot's URDF file to be loaded into the PyBullet simulation.
        """

        # Start pybullet physics engine without GUI and without gravity
        pyblt.connect(pyblt.DIRECT)
        pyblt.setGravity(0, 0, 0)


        pyblt.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_model = pyblt.loadURDF(urdf_path, useFixedBase=True, flags=pyblt.URDF_USE_SELF_COLLISION)

        # Disable joint motors
        for i in range(pyblt.getNumJoints(self.robot_model)):
            pyblt.setJointMotorControl2(self.robot_model, i, pyblt.VELOCITY_CONTROL, force=0)
    
        self.floor = pyblt.loadURDF("plane.urdf", useFixedBase=True)

        self.obstacles = []
    
    def check_self_collision(self) -> bool:
        """
        Checks whether the robot is in contact with itself (self-collision).

        Queries PyBullet for any contact points where the robot's own links 
        are touching each other based on the current joint configuration.

        Returns
        -------
        bool
            True if self-collision is detected; False otherwise.
        """

        body = self.robot_model
        contacts = pyblt.getContactPoints(bodyA=body, bodyB=body)
        return len(contacts) > 0

    def check_floor_collision(self) -> bool:
        """
        Checks whether any part of the robot (excluding the fixed base) is colliding with the floor.

        Retrieves contact points between the robot and the floor and ignores the 
        base link that is always expected to touch the ground.

        Returns
        -------
        bool
            True if more than one contact point exists, indicating a potential floor collision; False otherwise.
        """

        contacts = pyblt.getContactPoints(bodyA=self.robot_model, bodyB=self.floor)
        return len(contacts) > 1
    
    def check_obstacle_collision(self) -> bool:
        """
        Checks whether the robot is colliding with any user-defined obstacles in the environment.

        Iterates through the list of added obstacles and queries PyBullet for contact points
        between the robot and each obstacle.

        Returns
        -------
        bool
            True if at least one obstacle is in contact with the robot; False otherwise.
        """

        for obstacle in self.obstacles:
            contacts = pyblt.getContactPoints(bodyA=self.robot_model, bodyB=obstacle)
            if len(contacts) > 0:
                return True
        return False

    def check_any_collision(self) -> bool:
        """
        Performs a collision check across all possible sources.

        Evaluates self-collision, floor collision, and obstacle collision in sequence.
        Useful for determining whether the robot is in a valid, collision-free state.

        Returns
        -------
        bool
            True if any collision is detected; False if the robot is fully clear.
        """

        return self.check_self_collision() or self.check_floor_collision() or self.check_obstacle_collision()


    def set_angles(self, angles):
        """
        Sets the robot's movable joints to the specified target angles and steps the simulation.

        This method retrieves all non-fixed joints from the robot model and applies the provided
        joint angles accordingly. The number of angles must exactly match the number of movable joints.

        Parameters
        ----------
        angles : List[float]
            A list of joint angles (in radians) to apply to the robot's movable joints.

        Raises
        ------
        ValueError
            If the length of the input angle list does not match the number of movable joints.
        """
        # Get indices of movable joints (ignore fixed ones)
        movable_joints = [
            i for i in range(pyblt.getNumJoints(self.robot_model))
            if pyblt.getJointInfo(self.robot_model, i)[2] != pyblt.JOINT_FIXED
        ]

        if len(angles) != len(movable_joints):
            raise ValueError(f"Expected {len(movable_joints)} joint angles, but got {len(angles)}")

        for joint_index, angle in zip(movable_joints, angles):
            pyblt.resetJointState(self.robot_model, jointIndex=joint_index, targetValue=angle)

        pyblt.stepSimulation()
    
    def get_angles(self) -> List[float]:
        """
        Retrieves the current joint angles of all movable (non-fixed) joints in the robot.

        Iterates over the robot’s joint list, filtering out fixed joints, and 
        collects the position (angle) of each movable joint.

        Returns
        -------
        List[float]
            A list of joint angles (in radians) corresponding to the robot's movable joints.
        """
        # Get indices of movable joints (ignore fixed ones)
        movable_joints = [
            i for i in range(pyblt.getNumJoints(self.robot_model))
            if pyblt.getJointInfo(self.robot_model, i)[2] != pyblt.JOINT_FIXED
        ]

        angles = [pyblt.getJointState(self.robot_model, idx)[0] for idx in movable_joints]
        return angles



    def add_obstacle(self, pos: Tuple[float, float, float], type: str, size: float):
        """
        Adds a static obstacle to the simulation at the specified position.

        Supports spherical and box-shaped obstacles. These are created as fixed (massless) bodies 
        with collision shapes and registered in the internal obstacle list for future collision checks.

        Parameters
        ----------
        pos : Tuple[float, float, float]
            The (x, y, z) coordinates at which to place the center of the obstacle.
        type : str
            The shape of the obstacle to create: either "sphere" or "box".
        size : float
            Size of the obstacle: interpreted as radius for spheres or half-extent for boxes.

        Raises
        ------
        ValueError
            If the provided obstacle type is not supported (i.e., not "sphere" or "box").
        """

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
        """
        Removes an obstacle from the simulation based on its index in the internal list.

        Pops the obstacle from the internal list of registered bodies and deletes it from 
        the PyBullet simulation environment.

        Parameters
        ----------
        idx : int
            Index of the obstacle to remove in the list of added obstacles.

        Raises
        ------
        ValueError
            If the index is out of range or does not correspond to an existing obstacle.
        """

        if not 0 <= idx < len(self.obstacles):
            raise ValueError(f"Invalid index: No obstacle with index '{idx}'")

        body_id = self.obstacles.pop(idx)
        pyblt.removeBody(body_id) 
            
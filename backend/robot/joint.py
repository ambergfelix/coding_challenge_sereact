class Joint:
    """
    Represents a single joint in a robotic arm.

    Each joint has a name, a defined range of motion (minimum and maximum angles),
    and tracks its current angle. The angle is automatically clamped to remain within bounds.
    """

    def __init__(self, name: str, min_angle: float, max_angle: float):
        """
        Representation of a single revolute joint in a robotic arm.

        This class defines the motion constraints and current state of a joint,
        including functionality for clamping input angles to remain within physical limits.

        Attributes
        ----------
        name : str
            Name of the joint (e.g., 'Base', 'Shoulder')
        min_angle : float
            Minimum allowed angle for the joint in degrees
        max_angle : float
            Maximum allowed angle for the joint in degrees
        current_angle : float
            Current angle of the joint in degrees, clamped within joint limits
        """

        self.name = name
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = 0.0

    def set_angle(self, angle: float):
        """
        Sets the joint's current angle, automatically clamping to the joint's limits.

        Parameters
        ----------
        angle : float
            The target angle in degrees. If outside allowable range, it will be clamped.
        """
        self.current_angle = max(self.min_angle, min(self.max_angle, angle))

    def get_angle(self) -> float:
        """
        Returns the joint's current angle in degrees.

        Returns
        -------
        float
            The joint's current angle.
        """
        return self.current_angle

    def __repr__(self):
        """
        Returns a human-readable representation of the joint and its current angle.

        Returns
        -------
        str
            A formatted string showing the joint's name and angle in degrees.
        """
        return f"<Joint {self.name}: {self.current_angle:.1f} deg>"
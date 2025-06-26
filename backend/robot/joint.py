class Joint:
    def __init__(self, name: str, min_angle: float, max_angle: float):
        self.name = name
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = 0.0

    def set_angle(self, angle: float):
        # Clamp the input angle within the joint’s limits and update the current angle
        self.current_angle = max(self.min_angle, min(self.max_angle, angle))

    def get_angle(self) -> float:
        return self.current_angle

    def __repr__(self):
        # Provide a readable representation of the joint for debugging/logging
        return f"<Joint {self.name}: {self.current_angle:.1f} deg>"
from robot.joint import Joint
from typing import List



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
    
    def set_joint_angles(self, angles: List[float]):
        if len(angles) != len(self.joints):
            raise ValueError("Angle list does not match joint count.")
        for joint, angle in zip(self.joints, angles):
            joint.set_angle(angle)

    def get_joint_states(self) -> List[str]:
        return [repr(joint) for joint in self.joints]

    def __repr__(self):
        return "\n".join(self.get_joint_states())
    
    def test_six_axis_repr_output():
        robot = Six_axis_robot()
        # Set a known configuration
        test_angles = [10, -20, 30, -40, 50, -60]
        robot.set_joint_angles(test_angles)

        # Get the string representation
        repr_output = repr(robot)
        lines = repr_output.strip().split("\n")

        # Check that each line contains the corresponding joint name and angle
        expected_names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"]
        assert len(lines) == 6
        for name, angle, line in zip(expected_names, test_angles, lines):
            assert name in line
            assert f"{angle} deg" in line



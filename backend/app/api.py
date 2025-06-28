from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from backend.robot.six_axis_robot import SixAxisRobot
import backend.robot.motion_planning as motion
from typing import List
import numpy as np

app = FastAPI()
robot = SixAxisRobot()

# Allowed origins for CORS to enable frontend-backend interaction
origins = [
    "http://localhost:5173",
    "localhost:5173"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)

class JointAngles(BaseModel):
    """
    Data model representing target joint angles for the robot, 
    using Pydantic BaseModel for data parsing and validation.

    Attributes
    ----------
    angles : List[float]
        List of desired joint angles in degrees.
    """
    angles: List[float]


@app.post("/move")
def move_robot(data: JointAngles):
    """
    Moves the robot arm to a new joint configuration when a /move command is received from the client.

    It compares the robot’s current joint angles with the requested target angles.
    If the values differ beyond a small numerical tolerance, it generates a linearly interpolated
    motion path and executes the movement across all joints accordingly.

    Parameters
    ----------
    data : JointAngles
        Target joint angles for the robot.

    Returns
    -------
    dict
        Contains a status message and the final joint angles after execution.
    """
    curr_angles = robot.get_joint_angles()
    if np.allclose(curr_angles, data.angles, atol=1):
        return {
            "message": "Joint angles already at desired position",
            "angles": data.angles
        }

    path = motion.linear_interpolation(curr_angles, data.angles, 10)
    motion.execute_movement(robot, path)

    return {
        "message": "Joint angles updated",
        "angles": data.angles
    }
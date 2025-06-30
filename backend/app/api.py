from fastapi import FastAPI
from fastapi import WebSocket
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from backend.robot.six_axis_robot import SixAxisRobot
import backend.robot.motion_planning as motion
from typing import List
import numpy as np
import asyncio

app = FastAPI()
robot = SixAxisRobot()

# Allowed origins for CORS to enable frontend-backend interaction
origins = [
    "http://localhost:5173",
    "http://127.0.0.1:5173"
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
        List of desired joint angles in radians.
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
    if np.allclose(curr_angles, data.angles, atol=1e-3):
        return {
            "message": "Joint angles already at desired position",
            "angles": data.angles
        }

    # path = motion.linear_interpolation(curr_angles, data.angles, 400)
    print("Robot lim: ", robot.get_joint_limits())
    path = motion.cubic_spline_interpolation(curr_angles, data.angles, 100, robot.get_joint_limits())
    motion.execute_movement(robot, path)

    return {
        "message": "Joint angles updated",
        "angles": data.angles
    }

@app.websocket("/joints")
async def joint_stream(websocket: WebSocket):
    await websocket.accept()
    print("WebSocket connected")
    try:
        while True:
            # Retrieve current joint angles from your robot
            angles = robot.get_joint_angles()
            
            named_angles = {
                "shoulder_pan_joint": angles[0],
                "shoulder_lift_joint": angles[1],
                "elbow_joint": angles[2],
                "wrist_1_joint": angles[3],
                "wrist_2_joint": angles[4],
                "wrist_3_joint": angles[5]
            }

            # Send JSON to frontend
            await websocket.send_json(named_angles)

            # Wait before sending next update
            await asyncio.sleep(1/60)  # 200ms = 5Hz
    except Exception as e:
        print(f"WebSocket closed: {e}")


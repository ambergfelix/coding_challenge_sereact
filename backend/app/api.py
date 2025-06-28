from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from backend.robot.six_axis_robot import SixAxisRobot
import backend.robot.motion_planning as motion
from typing import List
import numpy as np




app = FastAPI()
robot = SixAxisRobot()


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
    angles: List[float]

@app.post("/move")
def move_robot(data: JointAngles):
    curr_angles = robot.get_joint_angles()
    if np.allclose(curr_angles, data.angles, atol=1):
        return {"message": "Joint angles already at desired position", "angles": data.angles}
    path = motion.linear_interpolation(curr_angles, data.angles, 10)
    motion.execute_movement(robot, path)
    return {"message": "Joint angles updated", "angles": data.angles}
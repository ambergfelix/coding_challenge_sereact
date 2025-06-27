from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from backend.robot.six_axis_robot import SixAxisRobot
from typing import List




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
    robot.set_joint_angles(data.angles)
    print("message recieved", data.angles)
    return {"message": "Joint angles updated", "angles": data.angles}
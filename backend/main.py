"""
Launcher script to concurrently start a FastAPI backend server and a Node.js frontend server.

This module initializes both servers using multiprocessing and opens the frontend in the user's default web browser.
"""

from pathlib import Path
import uvicorn
import utils.utils as util
import app.api as api
import json
import math

PI = math.pi

if __name__ == "__main__":
    """
    Launches the FastAPI backend API using uvicorn.

    The backend is served on host '0.0.0.0' and port 8000 with hot reload enabled for development.
    """
    # Load config
    config_path = Path(__file__).parent / "/robot-model/ur5/ur5_config.json"
    with open(config_path) as f:
        config = json.load(f)
    # Create robot instance
    robot = util.robot_from_config(config)
    inital_angles = [0, -PI/2, 0, 0, 0, 0]
    robot.set_joint_angles(inital_angles)
    api.app.state.robot = robot
    uvicorn.run("app.api:app", host="0.0.0.0", port=8000, reload=False)
"""
Launcher script to concurrently start a FastAPI backend server and a Node.js frontend server.

This module initializes both servers using multiprocessing and opens the frontend in the user's default web browser.
"""

import subprocess
import webbrowser
from pathlib import Path
import multiprocessing
import uvicorn

def start_backend():
    """
    Launches the FastAPI backend API using uvicorn.

    The backend is served on host '0.0.0.0' and port 8000 with hot reload enabled for development.
    """
    uvicorn.run("backend.app.api:app", host="0.0.0.0", port=8000, reload=False)

def start_frontend():
    """
    Launches the frontend development server using npm.

    The command is executed in the 'frontend' directory relative to this script.
    """
    frontend_path = Path(__file__).parent / "frontend"
    subprocess.run(["npm", "run", "dev"], cwd=frontend_path)

if __name__ == "__main__":
    # Start backend and frontend in parallel
    backend_process = multiprocessing.Process(target=start_backend)
    frontend_process = multiprocessing.Process(target=start_frontend)

    backend_process.start()
    frontend_process.start()

    # Open the browser to the frontend
    webbrowser.open_new_tab("http://localhost:5173")

    # Keep main process alive until both backend and frontend processes exit
    backend_process.join()
    frontend_process.join()
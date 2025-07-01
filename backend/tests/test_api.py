"""
Unit tests for the robot API endpoint responsible for joint movement commands.
"""

from fastapi.testclient import TestClient
from backend.app.api import app
import backend.app.api as api
import math

PI = math.pi

client = TestClient(app)

def test_move_robot(monkeypatch):
    """
    Test that the /move endpoint correctly triggers interpolated joint movement.

    Mocks the robot's current joint angle reading and the movement execution function
    to validate the internal logic of the API response. Ensures that interpolation 
    is used when the target angles differ from the current state.

    Parameters
    ----------
    monkeypatch : _pytest.monkeypatch.MonkeyPatch
        Pytest fixture used to override methods for testing with mock behavior.
    """
    # Create a mock robot with the required methods
    class MockRobot:
        def get_joint_angles(self):
            return [0.0, -PI/4, 0.0, 0.0, 0.0, 0.0]
        def get_joint_limits(self):
            return [(-PI, PI)] * 6

    # Attach the mock robot to the app state
    app.state.robot = MockRobot()
    
    def fake_execute_movement(robot_instance, path):
        assert isinstance(path, list)
        assert len(path) == 100

    # monkeypatch.setattr(api.robot, "get_joint_angles", fake_get_joint_angles)
    monkeypatch.setattr(api.motion, "execute_movement", fake_execute_movement)

    response = client.post("/move", json={"angles": [PI, -PI/2, PI/4, 0.0, -PI/4, PI/2]})

    assert response.status_code == 200
    assert response.json()["message"] == "Joint angles updated"
    response_angles = response.json()["angles"]
    assert len(response_angles) == 6
"""
Milestone 4A Tests: Gazebo Camera Controller & Endpoints Validation
Tests:
- CameraController unit logic (mode validation, command execution)
- GET /camera/mode endpoint
- POST /camera/mode endpoint (valid modes & error handling on invalid modes)
"""

import pytest
import asyncio
import pytest
from fastapi.testclient import TestClient

from api.camera import CameraController, CameraMode
from api.command_manager import CommandManager
from api.server import create_app
from tests.test_api_endpoints import MockRosAdapter


class DummyRunner:
    """Mock runner to capture Gazebo command invocations."""

    def __init__(self, succeed: bool = True):
        self.succeed = succeed
        self.calls = []

    def __call__(self, cmd: str, timeout: float = 3.0):
        self.calls.append(cmd)
        return self.succeed, "data: true" if self.succeed else "data: false"


@pytest.fixture
def dummy_runner():
    return DummyRunner(succeed=True)


@pytest.fixture
def camera_controller(dummy_runner):
    controller = CameraController()
    controller._execute_gz = dummy_runner
    return controller


@pytest.fixture
def client(camera_controller):
    adapter = MockRosAdapter(healthy=True)
    manager = CommandManager()
    app = create_app(
        adapter=adapter,
        manager=manager,
        camera=camera_controller,
    )
    return TestClient(app)


def test_camera_controller_modes(camera_controller, dummy_runner):
    """Test setting all supported camera modes via the CameraController directly."""
    # Test Follow Mode
    success, msg = asyncio.run(camera_controller.set_mode(CameraMode.FOLLOW))
    assert success is True
    assert camera_controller.current_mode == CameraMode.FOLLOW.value
    assert any("/gui/follow" in c and "four_wheel_robot" in c for c in dummy_runner.calls)
    assert any("/gui/follow/offset" in c for c in dummy_runner.calls)

    dummy_runner.calls.clear()

    # Test Top-Down Mode
    success, msg = asyncio.run(camera_controller.set_mode(CameraMode.TOP_DOWN))
    assert success is True
    assert camera_controller.current_mode == CameraMode.TOP_DOWN.value
    assert any('/gui/follow' in c and '""' in c for c in dummy_runner.calls)
    assert any("/gui/move_to/pose" in c for c in dummy_runner.calls)

    dummy_runner.calls.clear()

    # Test Free Orbit Mode
    success, msg = asyncio.run(camera_controller.set_mode(CameraMode.FREE))
    assert success is True
    assert camera_controller.current_mode == CameraMode.FREE.value
    assert any('/gui/follow' in c and '""' in c for c in dummy_runner.calls)

    dummy_runner.calls.clear()

    # Test Reset Mode
    success, msg = asyncio.run(camera_controller.set_mode(CameraMode.RESET))
    assert success is True
    assert camera_controller.current_mode == CameraMode.RESET.value
    assert any('/gui/follow' in c and '""' in c for c in dummy_runner.calls)
    assert any("/gui/move_to/pose" in c for c in dummy_runner.calls)


def test_camera_controller_invalid_mode(camera_controller):
    """Test that invalid camera modes are rejected."""
    success, msg = asyncio.run(camera_controller.set_mode("cinematic_flythrough"))
    assert success is False
    assert "Invalid camera mode" in msg


def test_get_camera_mode_endpoint(client):
    """Test GET /camera/mode returns current mode and list of available modes."""
    response = client.get("/camera/mode")
    assert response.status_code == 200
    data = response.json()
    assert "current_mode" in data
    assert "available_modes" in data
    assert data["available_modes"] == ["follow", "top_down", "free", "reset"]
    assert "descriptions" in data


def test_post_camera_mode_valid(client, dummy_runner):
    """Test POST /camera/mode sets camera modes properly."""
    for mode in ["follow", "top_down", "free", "reset"]:
        response = client.post("/camera/mode", json={"mode": mode})
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
        assert data["current_mode"] == mode
        assert "message" in data


def test_post_camera_mode_invalid(client):
    """Test POST /camera/mode rejects invalid modes with 400 Bad Request."""
    response = client.post("/camera/mode", json={"mode": "fpv_drone"})
    assert response.status_code == 400
    data = response.json()
    assert "Invalid camera mode" in data["detail"]

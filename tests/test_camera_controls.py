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


def test_office_geometry_occlusion_detection():
    """Test OfficeGeometry raycasting correctly detects wall obstructions."""
    from api.camera import OfficeGeometry
    geo = OfficeGeometry()
    assert len(geo.walls) >= 15

    # Known wall: wall_board_east around x=-22.08, y=10.88, z=0..2.6
    # Line of sight across wall: camera at x=-20.0, y=10.0, z=2.0 -> robot at x=-24.0, y=10.0, z=0.15
    occluded, wall_name = geo.is_occluded([-20.0, 10.0, 2.0], [-24.0, 10.0, 0.15])
    assert occluded is True
    assert wall_name is not None

    # Clear line of sight within room: camera at x=-24.0, y=9.0, z=3.0 -> robot at x=-24.0, y=10.0, z=0.15
    occluded_clear, _ = geo.is_occluded([-24.0, 9.0, 3.0], [-24.0, 10.0, 0.15])
    assert occluded_clear is False


def test_adaptive_offset_evaluation_boardroom(camera_controller):
    """Test that candidate offset evaluation picks an unobstructed steep/close view in tight spaces."""
    # Robot inside Board Room near east wall: rx = -22.9, ry = 9.24, ryaw = 2.98
    name, offset = camera_controller.evaluate_best_offset(-22.9, 9.24, 2.98)
    assert offset is not None
    # Must pick an offset that provides clear visibility
    rx, ry, ryaw = -22.9, 9.24, 2.98
    import math
    wx = rx + offset[0] * math.cos(ryaw) - offset[1] * math.sin(ryaw)
    wy = ry + offset[0] * math.sin(ryaw) + offset[1] * math.cos(ryaw)
    wz = offset[2]
    occ, _ = camera_controller.geometry.is_occluded([wx, wy, wz], [rx, ry, 0.15])
    assert occ is False


def test_adaptive_offset_evaluation_open_corridor(camera_controller):
    """Test that candidate offset evaluation keeps preferred elevated wide chase in open corridor."""
    # Robot in open West Corridor: rx = -25.0, ry = 7.25, ryaw = 3.1415
    name, offset = camera_controller.evaluate_best_offset(-25.0, 7.25, 3.1415)
    assert name == "elevated_wide_chase"
    assert offset[0] == -1.2
    assert offset[2] == 3.8



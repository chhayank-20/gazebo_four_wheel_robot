"""
Stage A Unit Tests: FastAPI Endpoint & Logic Validation (with Mocked ROS Adapter)
Verifies:
- Whitelist enforcement (reject unknown locations with 422)
- Concurrency limit (reject concurrent commands with 409)
- Freshness checks (stale/missing sensors return None/null, not 0)
- Command status & progress tracking
- Cancellation flow & return to idle
"""

import time
import pytest
from fastapi.testclient import TestClient

from api.command_manager import CommandManager, CommandStatus
from api.destinations import DESTINATIONS
from api.server import create_app


class MockRosAdapter:
    """Mock implementation of RosAdapter for unit testing without live ROS 2."""

    def __init__(self, healthy: bool = True):
        self.healthy = healthy
        self.nav2_ready = healthy
        self.odom_fresh = healthy
        self.scan_fresh = healthy
        self.tf_fresh = healthy
        self.dispatched_goals = []
        self._is_busy = False
        self._current_cmd_id = None
        self._done_callback = None
        self._feedback_callback = None

    def start(self) -> bool:
        return True

    def stop(self):
        pass

    def is_nav2_ready(self) -> bool:
        return self.nav2_ready

    def is_busy(self) -> bool:
        return self._is_busy

    def get_health_status(self):
        return {
            "ros2_connected": self.healthy,
            "nav2_action_server_ready": self.nav2_ready,
            "odom_active": self.odom_fresh,
            "scan_active": self.scan_fresh,
            "tf_active": self.tf_fresh,
            "overall_status": "healthy" if self.healthy else "degraded",
        }

    def get_current_pose(self):
        if not self.tf_fresh:
            return None
        return {
            "x": -2.35,
            "y": 6.85,
            "yaw": 0.0,
            "frame_id": "map",
            "timestamp": time.time(),
        }

    def get_current_velocity(self):
        if not self.odom_fresh:
            return None
        return {
            "linear_x": 0.05,
            "angular_z": 0.01,
            "timestamp": time.time(),
        }

    def get_min_lidar_clearance(self):
        if not self.scan_fresh:
            return None
        return {
            "min_lidar_m": 1.25,
            "timestamp": time.time(),
        }

    def send_navigation_goal(
        self,
        command_id: str,
        location_id: str,
        waypoints,
        feedback_callback=None,
        done_callback=None,
    ) -> bool:
        if not self.nav2_ready:
            return False
        self._is_busy = True
        self._current_cmd_id = command_id
        self._feedback_callback = feedback_callback
        self._done_callback = done_callback
        self.dispatched_goals.append((command_id, location_id, waypoints))
        return True

    def simulate_feedback(self, dist: float, rec: int, poses: int):
        if self._feedback_callback:
            self._feedback_callback({
                "command_id": self._current_cmd_id,
                "distance_remaining_m": dist,
                "recoveries": rec,
                "poses_remaining": poses,
            })

    def simulate_success(self):
        self._is_busy = False
        if self._done_callback:
            self._done_callback(self._current_cmd_id, 4, "Success")

    def cancel_active_goal(self, timeout_sec: float = 3.0):
        if not self._is_busy:
            return False, "No active goal"
        self._is_busy = False
        if self._done_callback:
            self._done_callback(self._current_cmd_id, 5, "Canceled")
        return True, f"Command {self._current_cmd_id} canceled successfully; stationary rest verified"


@pytest.fixture
def mock_adapter():
    return MockRosAdapter(healthy=True)


@pytest.fixture
def cmd_manager():
    return CommandManager()


@pytest.fixture
def client(mock_adapter, cmd_manager):
    app = create_app(adapter=mock_adapter, manager=cmd_manager)
    return TestClient(app)


def test_health_endpoint_healthy(client):
    res = client.get("/health")
    assert res.status_code == 200
    data = res.json()
    assert data["overall_status"] == "healthy"
    assert data["ros2_connected"] is True
    assert data["nav2_action_server_ready"] is True
    assert data["odom_active"] is True
    assert data["scan_active"] is True
    assert data["tf_active"] is True


def test_health_endpoint_degraded():
    mock = MockRosAdapter(healthy=False)
    app = create_app(adapter=mock, manager=CommandManager())
    c = TestClient(app)
    res = c.get("/health")
    assert res.status_code == 200
    data = res.json()
    assert data["overall_status"] == "degraded"
    assert data["ros2_connected"] is False


def test_status_endpoint_fresh(client):
    res = client.get("/status")
    assert res.status_code == 200
    data = res.json()
    assert data["overall_status"] == "healthy"
    assert data["active_command"] is None
    assert data["pose"]["x"] == -2.35
    assert data["velocity"]["linear_x"] == 0.05
    assert data["lidar"]["min_lidar_m"] == 1.25


def test_status_endpoint_stale_feeds():
    mock = MockRosAdapter(healthy=True)
    mock.odom_fresh = False
    mock.scan_fresh = False
    mock.tf_fresh = False
    app = create_app(adapter=mock, manager=CommandManager())
    c = TestClient(app)

    res = c.get("/status")
    assert res.status_code == 200
    data = res.json()
    # Stale feeds MUST be null / None, not 0
    assert data["pose"] is None
    assert data["velocity"] is None
    assert data["lidar"] is None


def test_locations_endpoint(client):
    res = client.get("/locations")
    assert res.status_code == 200
    data = res.json()
    locations = data["locations"]
    assert len(locations) == len(DESTINATIONS)
    loc_ids = [l["id"] for l in locations]
    assert "lobby" in loc_ids
    assert "living_demo" in loc_ids
    assert "board_room" in loc_ids
    assert "accounts_ws" in loc_ids


def test_navigate_rejects_unknown_location(client):
    res = client.post("/navigate", json={"location_id": "secret_room_404"})
    assert res.status_code == 422
    assert "Unknown location" in res.json()["detail"]


def test_navigate_rejects_arbitrary_coordinates(client):
    # API schema accepts location_id, not arbitrary x, y coordinates
    res = client.post("/navigate", json={"x": 100.0, "y": 200.0})
    assert res.status_code == 422


def test_navigate_success_and_status_tracking(client, mock_adapter):
    res = client.post("/navigate", json={"location_id": "living_demo"})
    assert res.status_code == 202
    data = res.json()
    cmd_id = data["command_id"]
    assert cmd_id.startswith("cmd_")
    assert data["location_id"] == "living_demo"
    assert data["status"] in ("accepted", "running")

    # Verify status endpoint reflects active command
    st_res = client.get("/status")
    st_data = st_res.json()
    assert st_data["active_command"] is not None
    assert st_data["active_command"]["command_id"] == cmd_id

    # Verify feedback simulation
    mock_adapter.simulate_feedback(dist=5.2, rec=0, poses=2)
    cmd_res = client.get(f"/commands/{cmd_id}")
    assert cmd_res.status_code == 200
    cmd_data = cmd_res.json()
    assert cmd_data["distance_remaining_m"] == 5.2
    assert cmd_data["poses_remaining"] == 2

    # Simulate success
    mock_adapter.simulate_success()
    cmd_res2 = client.get(f"/commands/{cmd_id}")
    cmd_data2 = cmd_res2.json()
    assert cmd_data2["status"] == "succeeded"
    assert cmd_data2["distance_remaining_m"] == 0.0

    # Robot should no longer be busy
    st_res2 = client.get("/status")
    assert st_res2.json()["active_command"] is None


def test_navigate_concurrency_lock(client):
    # Start first command
    res1 = client.post("/navigate", json={"location_id": "living_demo"})
    assert res1.status_code == 202

    # Attempt second concurrent command
    res2 = client.post("/navigate", json={"location_id": "board_room"})
    assert res2.status_code == 409
    assert "already navigating" in res2.json()["detail"]


def test_navigate_server_unavailable():
    mock = MockRosAdapter(healthy=True)
    mock.nav2_ready = False
    app = create_app(adapter=mock, manager=CommandManager())
    c = TestClient(app)

    res = c.post("/navigate", json={"location_id": "living_demo"})
    assert res.status_code == 503
    assert "Nav2 action server is not ready" in res.json()["detail"]


def test_command_not_found(client):
    res = client.get("/commands/cmd_non_existent")
    assert res.status_code == 404


def test_cancellation_flow(client, mock_adapter):
    # Dispatch navigation
    res = client.post("/navigate", json={"location_id": "board_room"})
    assert res.status_code == 202
    cmd_id = res.json()["command_id"]

    # Cancel command
    cancel_res = client.post(f"/commands/{cmd_id}/cancel")
    assert cancel_res.status_code == 200
    cancel_data = cancel_res.json()
    assert cancel_data["command_id"] == cmd_id
    assert cancel_data["status"] == "canceled"
    assert cancel_data["verified_stopped"] is True

    # Verify command status is canceled
    cmd_res = client.get(f"/commands/{cmd_id}")
    assert cmd_res.json()["status"] == "canceled"

    # Verify robot is free to accept a new command now
    res_next = client.post("/navigate", json={"location_id": "lobby"})
    assert res_next.status_code == 202

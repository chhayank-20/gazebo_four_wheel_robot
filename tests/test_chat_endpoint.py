"""
Milestone 2 Unit Tests: LLM-Powered Text Chat Endpoint & Safety Tool Execution
Verifies:
- Ordinary conversational messages (no tool calls)
- Telemetry & destination queries (get_robot_status, list_locations)
- Valid navigation requests with approved destination IDs
- Ambiguity detection & required clarification (e.g. 'experience room')
- Unknown / unapproved destination rejection
- Active command status queries & safe cancellation
- Concurrency limit (rejection of navigation when robot is busy)
- Upstream LLM provider timeout (504) and failure (502) error handling
- Core Robot Control API resilience when LLM provider is down
- Server-side tool validation (rejection of unsupported tools / arbitrary code)
"""

import json
import time
import pytest
from fastapi.testclient import TestClient

from api.command_manager import CommandManager, CommandStatus
from api.destinations import DESTINATIONS
from api.llm_service import LLMService, MockLLMProvider
from api.server import create_app
from api.tools import execute_tool


class MockRosAdapter:
    """Mock implementation of RosAdapter for chat testing."""

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

    def cancel_active_goal(self, timeout_sec: float = 3.0):
        self._is_busy = False
        self._current_cmd_id = None
        return True, "Active Nav2 goal canceled successfully."


@pytest.fixture
def test_setup():
    adapter = MockRosAdapter(healthy=True)
    manager = CommandManager()
    provider = MockLLMProvider()
    service = LLMService(ros_adapter=adapter, command_manager=manager, provider=provider)
    app = create_app(adapter=adapter, manager=manager, service=service)
    client = TestClient(app)
    return client, adapter, manager, service, provider


# 1. Ordinary Conversational Messages
def test_chat_greeting_no_tools(test_setup):
    client, _, _, _, _ = test_setup
    resp = client.post("/chat", json={"message": "Hello! Who are you?"})
    assert resp.status_code == 200
    data = resp.json()
    assert "assistant" in data["response"].lower() or "robot" in data["response"].lower()
    assert data["tool_calls"] == []
    assert data["command_id"] is None
    assert data["conversation_id"] == "default"


def test_chat_empty_message_validation(test_setup):
    client, _, _, _, _ = test_setup
    resp = client.post("/chat", json={"message": "   "})
    assert resp.status_code == 422


# 2. Status & Location Queries
def test_chat_status_query(test_setup):
    client, adapter, _, _, _ = test_setup
    resp = client.post("/chat", json={"message": "What is the robot's current status?"})
    assert resp.status_code == 200
    data = resp.json()
    assert len(data["tool_calls"]) == 1
    tc = data["tool_calls"][0]
    assert tc["tool_name"] == "get_robot_status"
    assert tc["result"]["overall_status"] == "healthy"
    assert "healthy" in data["response"].lower() or "pose" in data["response"].lower()


def test_chat_list_locations(test_setup):
    client, _, _, _, _ = test_setup
    resp = client.post("/chat", json={"message": "Where can you take me? List the destinations."})
    assert resp.status_code == 200
    data = resp.json()
    assert len(data["tool_calls"]) == 1
    tc = data["tool_calls"][0]
    assert tc["tool_name"] == "list_locations"
    assert len(tc["result"]["approved_locations"]) == len(DESTINATIONS)
    assert "lobby" in data["response"].lower() or "board room" in data["response"].lower()


# 3. Valid Navigation Requests
def test_chat_navigate_to_valid_location(test_setup):
    client, adapter, manager, _, _ = test_setup
    resp = client.post("/chat", json={"message": "Please take me to the Lobby"})
    assert resp.status_code == 200
    data = resp.json()

    # Tool call check
    assert len(data["tool_calls"]) == 1
    tc = data["tool_calls"][0]
    assert tc["tool_name"] == "navigate_to_location"
    assert tc["arguments"]["location_id"] == "lobby"
    assert tc["result"]["status"] in ("accepted", "running")
    cmd_id = tc["result"]["command_id"]
    assert cmd_id is not None
    assert data["command_id"] == cmd_id

    # Verify adapter dispatched goal
    assert len(adapter.dispatched_goals) == 1
    assert adapter.dispatched_goals[0][1] == "lobby"

    # Truthfulness check: Response must NOT claim arrival
    resp_text = data["response"].lower()
    assert "arrived at" not in resp_text
    assert any(w in resp_text for w in ["dispatched", "started", "en route", "navigating", "underway", "accepted"])


# 4. Ambiguity Detection & Clarification
def test_chat_ambiguous_experience_room(test_setup):
    client, adapter, _, _, _ = test_setup
    resp = client.post("/chat", json={"message": "Take me to the experience room"})
    assert resp.status_code == 200
    data = resp.json()

    # Ambiguity must NOT trigger a navigation tool call!
    assert data["tool_calls"] == []
    assert data["command_id"] is None
    assert len(adapter.dispatched_goals) == 0

    # Assistant must clarify between the three experience rooms
    resp_text = data["response"].lower()
    assert "living" in resp_text or "hotel" in resp_text or "hospital" in resp_text
    assert "which" in resp_text or "specify" in resp_text or "three" in resp_text


# 5. Unknown / Malformed Destination Handling
def test_chat_unknown_destination(test_setup):
    client, adapter, _, _, _ = test_setup
    resp = client.post("/chat", json={"message": "Drive to the cafeteria on floor 3"})
    assert resp.status_code == 200
    data = resp.json()

    # Either no tool call was made or tool returned error
    if data["tool_calls"]:
        tc = data["tool_calls"][0]
        assert "error" in tc["result"]
    assert data["command_id"] is None
    assert len(adapter.dispatched_goals) == 0

    resp_text = data["response"].lower()
    assert "approved" in resp_text or "unknown" in resp_text or "cannot" in resp_text or "valid" in resp_text


# 6. Cancellation & Command Status
def test_chat_cancellation(test_setup):
    client, adapter, manager, _, _ = test_setup

    # First start a navigation
    nav_resp = client.post("/chat", json={"message": "Navigate to the board room"})
    cmd_id = nav_resp.json()["command_id"]
    assert cmd_id is not None
    assert manager.is_busy()

    # Now ask to stop
    cancel_resp = client.post("/chat", json={"message": "Stop the robot immediately!"})
    assert cancel_resp.status_code == 200
    cancel_data = cancel_resp.json()

    assert len(cancel_data["tool_calls"]) == 1
    tc = cancel_data["tool_calls"][0]
    assert tc["tool_name"] == "cancel_navigation"
    assert tc["result"]["verified_stopped"] is True
    assert not manager.is_busy()
    assert "canceled" in cancel_data["response"].lower() or "stopped" in cancel_data["response"].lower()


def test_chat_command_status_query(test_setup):
    client, adapter, manager, _, _ = test_setup
    nav_resp = client.post("/chat", json={"message": "Go to hotel_exp"})
    cmd_id = nav_resp.json()["command_id"]

    status_resp = client.post("/chat", json={"message": f"Check status of command {cmd_id}"})
    assert status_resp.status_code == 200
    status_data = status_resp.json()

    assert len(status_data["tool_calls"]) == 1
    tc = status_data["tool_calls"][0]
    assert tc["tool_name"] == "get_command_status"
    assert tc["result"]["command_id"] == cmd_id
    assert tc["result"]["status"] == "running"


# 7. Concurrency / Duplicate Prevention
def test_chat_prevent_duplicate_navigation_when_busy(test_setup):
    client, adapter, manager, _, _ = test_setup

    # First navigation
    resp1 = client.post("/chat", json={"message": "Go to lobby"})
    assert resp1.status_code == 200
    cmd_id1 = resp1.json()["command_id"]
    assert cmd_id1 is not None

    # Second navigation while first is running
    resp2 = client.post("/chat", json={"message": "Go to board room"})
    assert resp2.status_code == 200
    data2 = resp2.json()

    # Tool call should have returned busy error
    assert len(data2["tool_calls"]) == 1
    tc = data2["tool_calls"][0]
    assert "error" in tc["result"]
    assert "busy" in tc["result"]["error"].lower()

    # Must only have dispatched one goal to adapter
    assert len(adapter.dispatched_goals) == 1
    assert adapter.dispatched_goals[0][1] == "lobby"


# 8. Provider Timeouts and Errors
def test_chat_provider_timeout_504():
    adapter = MockRosAdapter(healthy=True)
    manager = CommandManager()
    timeout_provider = MockLLMProvider(should_timeout=True)
    service = LLMService(ros_adapter=adapter, command_manager=manager, provider=timeout_provider)
    app = create_app(adapter=adapter, manager=manager, service=service)
    client = TestClient(app)

    resp = client.post("/chat", json={"message": "Hello robot"})
    assert resp.status_code == 504
    assert "timed out" in resp.json()["detail"].lower()

    # Verify core Robot Control API remains functional
    health_resp = client.get("/health")
    assert health_resp.status_code == 200
    assert health_resp.json()["overall_status"] == "healthy"


def test_chat_provider_error_502():
    adapter = MockRosAdapter(healthy=True)
    manager = CommandManager()
    failing_provider = MockLLMProvider(should_fail=True)
    service = LLMService(ros_adapter=adapter, command_manager=manager, provider=failing_provider)
    app = create_app(adapter=adapter, manager=manager, service=service)
    client = TestClient(app)

    resp = client.post("/chat", json={"message": "Hello robot"})
    assert resp.status_code == 502
    assert "service error" in resp.json()["detail"].lower()

    # Verify /locations endpoint still works
    loc_resp = client.get("/locations")
    assert loc_resp.status_code == 200
    assert len(loc_resp.json()["locations"]) == len(DESTINATIONS)


# 9. Server-side Tool Allowlist and Malicious/Unsupported Rejection
def test_execute_tool_unsupported():
    adapter = MockRosAdapter(healthy=True)
    manager = CommandManager()

    # Try executing an arbitrary command or unknown tool
    result = execute_tool(
        tool_name="system_exec",
        arguments={"cmd": "rm -rf /"},
        ros_adapter=adapter,
        command_manager=manager,
    )
    assert "error" in result
    assert "system_exec" in result["error"]
    assert "unauthorized" in result["error"] or "unsupported" in result["error"]


def test_execute_tool_invalid_location():
    adapter = MockRosAdapter(healthy=True)
    manager = CommandManager()

    result = execute_tool(
        tool_name="navigate_to_location",
        arguments={"location_id": "secret_vault"},
        ros_adapter=adapter,
        command_manager=manager,
    )
    assert "error" in result
    assert "secret_vault" in result["error"]
    assert "Approved locations" in result["error"]

"""
Milestone 3 Unit Tests: Web Chat UI, Static Assets & Secret Safety
Verifies:
- Root GET / serves index.html with 200 OK and expected HTML tags
- Static assets /static/style.css and /static/app.js are served with 200 OK
- No API keys or secrets are exposed in client HTML/JS files
- UI endpoints integrate with /chat, /status, /locations, /health
"""

import os
import pytest
from fastapi.testclient import TestClient

from api.command_manager import CommandManager
from api.destinations import DESTINATIONS
from api.llm_service import LLMService, MockLLMProvider
from api.server import STATIC_DIR, create_app


class MockRosAdapter:
    def __init__(self, healthy: bool = True):
        self.healthy = healthy

    def start(self) -> bool:
        return True

    def stop(self):
        pass

    def is_nav2_ready(self) -> bool:
        return self.healthy

    def is_busy(self) -> bool:
        return False

    def get_health_status(self):
        return {
            "ros2_connected": self.healthy,
            "nav2_action_server_ready": self.healthy,
            "odom_active": self.healthy,
            "scan_active": self.healthy,
            "tf_active": self.healthy,
            "overall_status": "healthy" if self.healthy else "degraded",
        }

    def get_current_pose(self):
        return {"x": -1.8, "y": 0.0, "yaw": 1.5708, "frame_id": "map", "timestamp": 10.0}

    def get_current_velocity(self):
        return {"linear_x": 0.0, "angular_z": 0.0, "timestamp": 10.0}

    def get_min_lidar_clearance(self):
        return {"min_lidar_m": 1.25, "timestamp": 10.0}


@pytest.fixture
def client():
    adapter = MockRosAdapter(healthy=True)
    manager = CommandManager()
    provider = MockLLMProvider()
    service = LLMService(ros_adapter=adapter, command_manager=manager, provider=provider)
    app = create_app(adapter=adapter, manager=manager, service=service)
    return TestClient(app)


def test_root_serves_html(client):
    resp = client.get("/")
    assert resp.status_code == 200
    assert "text/html" in resp.headers.get("content-type", "")
    assert "<!DOCTYPE html>" in resp.text
    assert "Office Robot Assistant" in resp.text
    assert "Interactive Robot Chat" in resp.text
    assert "messages-container" in resp.text
    assert "telemetry-sidebar" in resp.text


def test_static_stylesheet_served(client):
    resp = client.get("/static/style.css")
    assert resp.status_code == 200
    assert "text/css" in resp.headers.get("content-type", "")
    assert "--bg-app:" in resp.text
    assert ".message-bubble" in resp.text


def test_static_javascript_served(client):
    resp = client.get("/static/app.js")
    assert resp.status_code == 200
    assert "javascript" in resp.headers.get("content-type", "")
    assert "submitUserMessage" in resp.text
    assert "fetchRobotHealth" in resp.text
    assert "fetchRobotStatus" in resp.text


def test_no_secrets_exposed_in_client_files():
    """Verify that no sensitive environment variables or keys are leaked in client code."""
    for filename in ["index.html", "style.css", "app.js"]:
        path = os.path.join(STATIC_DIR, filename)
        assert os.path.exists(path), f"File {filename} must exist"
        with open(path, "r", encoding="utf-8") as f:
            content = f.read()

        # Check for leaked keys or placeholders
        assert "LLM_API_KEY" not in content
        assert "OPENAI_API_KEY" not in content
        assert "sk-" not in content
        assert "gsk_" not in content


def test_frontend_supported_endpoints(client):
    """Verify that all endpoints expected by app.js respond with expected JSON contracts."""
    # 1. Health
    h = client.get("/health")
    assert h.status_code == 200
    assert "overall_status" in h.json()

    # 2. Status
    s = client.get("/status")
    assert s.status_code == 200
    assert "pose" in s.json()
    assert "velocity" in s.json()
    assert "lidar" in s.json()

    # 3. Locations
    l = client.get("/locations")
    assert l.status_code == 200
    assert len(l.json()["locations"]) == len(DESTINATIONS)

    # 4. Chat
    c = client.post("/chat", json={"message": "Hello robot!"})
    assert c.status_code == 200
    assert "response" in c.json()
    assert "tool_calls" in c.json()

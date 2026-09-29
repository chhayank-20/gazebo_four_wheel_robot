#!/usr/bin/env python3
"""
Audit Test Script for Requirement 5:
- 5A: Stale sensor stream detection (pause bridge -> feeds become stale -> status reports degraded and null fields -> resume bridge -> healthy)
- 5B: Unavailable Nav2 server reporting (stops bt_navigator -> health reports nav2 unavailable -> POST /navigate returns 503 Service Unavailable -> resumes bt_navigator)
"""

import json
import subprocess
import sys
import time
import urllib.request
import urllib.error

BASE_URL = "http://localhost:8000"


def http_get(path: str):
    req = urllib.request.Request(f"{BASE_URL}{path}", headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            data = json.loads(resp.read().decode())
            return resp.status, data
    except urllib.error.HTTPError as e:
        data = json.loads(e.read().decode()) if e.headers.get_content_type() == "application/json" else {"detail": str(e)}
        return e.code, data
    except Exception as e:
        return 0, {"error": str(e)}


def http_post(path: str, payload: dict):
    body = json.dumps(payload).encode()
    req = urllib.request.Request(f"{BASE_URL}{path}", data=body, headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            data = json.loads(resp.read().decode())
            return resp.status, data
    except urllib.error.HTTPError as e:
        try:
            data = json.loads(e.read().decode())
        except Exception:
            data = {"detail": str(e)}
        return e.code, data
    except Exception as e:
        return 0, {"error": str(e)}


def run_container_cmd(cmd: list):
    full_cmd = ["docker", "exec", "ros2_gazebo_robot"] + cmd
    res = subprocess.run(full_cmd, capture_output=True, text=True)
    return res.returncode, res.stdout, res.stderr


def test_degraded_scenarios():
    print("=" * 75)
    print("AUDIT REQUIREMENT 5: STALE SENSORS & UNAVAILABLE NAV2 REPORTING")
    print("=" * 75)

    # 1. Baseline health
    code, health = http_get("/health")
    assert code == 200 and health.get("overall_status") == "healthy", f"Initial system not healthy: {health}"
    print(f"[Init] Baseline health verified: {health}")

    # -------------------------------------------------------------
    # 5A: Stale Sensor Stream Test (SIGSTOP on ros_gz_bridge)
    # -------------------------------------------------------------
    print("\n--- Test 5A: Stale sensor stream verification ---")
    print(" -> Pausing ros_gz_bridge with SIGSTOP to simulate sensor feed stoppage...")
    run_container_cmd(["pkill", "-STOP", "-f", "parameter_bridge"])

    # Wait past the 2.0s freshness timeout
    print(" -> Waiting 3.0s for freshness timeout to trigger...")
    time.sleep(3.0)

    # Check /health reports degraded
    code, degraded_health = http_get("/health")
    print(f" -> Degraded health: code={code}, body={degraded_health}")
    assert code == 200
    assert degraded_health["odom_active"] is False, "odom_active should be False"
    assert degraded_health["scan_active"] is False, "scan_active should be False"
    assert degraded_health["overall_status"] == "degraded", f"Expected degraded, got {degraded_health['overall_status']}"

    # Check /status reports null for stale fields (not 0)
    code, degraded_status = http_get("/status")
    print(f" -> Degraded status: code={code}, velocity={degraded_status.get('velocity')}, lidar={degraded_status.get('lidar')}")
    assert code == 200
    assert degraded_status["velocity"] is None, "velocity MUST be null when stale"
    assert degraded_status["lidar"] is None, "lidar MUST be null when stale"
    print(" -> [PASS] Stale sensor feeds correctly reported as degraded and null.")

    # Resume ros_gz_bridge
    print(" -> Resuming ros_gz_bridge with SIGCONT...")
    run_container_cmd(["pkill", "-CONT", "-f", "parameter_bridge"])
    time.sleep(1.5)

    # Verify recovery to healthy
    code, recovered_health = http_get("/health")
    print(f" -> Recovered health: code={code}, body={recovered_health}")
    assert code == 200 and recovered_health["overall_status"] == "healthy"
    assert recovered_health["odom_active"] is True
    assert recovered_health["scan_active"] is True
    print(" -> [PASS] Test 5A: Stale sensor stream handling & recovery verified.")

    # -------------------------------------------------------------
    # 5B: Unavailable Nav2 Server Test (SIGSTOP on bt_navigator)
    # -------------------------------------------------------------
    print("\n--- Test 5B: Unavailable Nav2 server verification ---")
    print(" -> Pausing bt_navigator with SIGSTOP to simulate Nav2 server unavailability...")
    run_container_cmd(["pkill", "-STOP", "-f", "bt_navigator"])
    time.sleep(1.5)

    # Check /health reports nav2_action_server_ready: False
    code, nav_down_health = http_get("/health")
    print(f" -> Health with Nav2 paused: code={code}, body={nav_down_health}")
    assert code == 200
    assert nav_down_health["nav2_action_server_ready"] is False
    assert nav_down_health["overall_status"] in ("degraded", "unhealthy")

    # Check that POST /navigate rejects requests with 503 Service Unavailable
    code, nav_reject = http_post("/navigate", {"location_id": "living_demo"})
    print(f" -> POST /navigate with Nav2 paused: code={code}, body={nav_reject}")
    assert code == 503, f"Expected 503 Service Unavailable, got {code}"
    assert "Nav2 action server is not ready" in nav_reject.get("detail", "")
    print(" -> [PASS] POST /navigate correctly rejected with 503 Service Unavailable.")

    # Resume bt_navigator
    print(" -> Resuming bt_navigator with SIGCONT...")
    run_container_cmd(["pkill", "-CONT", "-f", "bt_navigator"])
    time.sleep(2.0)

    # Verify recovery
    code, recovered_nav = http_get("/health")
    print(f" -> Recovered health after Nav2 resumed: code={code}, body={recovered_nav}")
    assert code == 200 and recovered_nav["nav2_action_server_ready"] is True
    assert recovered_nav["overall_status"] == "healthy"
    print(" -> [PASS] Test 5B: Unavailable Nav2 server handling & recovery verified.")

    print("\n" + "=" * 75)
    print("ALL AUDIT REQUIREMENT 5 TESTS PASSED SUCCESSFULLY!")
    print("=" * 75)


if __name__ == "__main__":
    test_degraded_scenarios()

#!/usr/bin/env python3
"""
Stage B Live Gazebo Integration Test: Robot Control API
Tests against live Gazebo + Nav2 + FastAPI service:
1. Health & readiness validation (/health, /status, /locations)
2. Live navigation to Living Demo with progress monitoring
3. Concurrency lock validation (409 Conflict during active navigation)
4. Safe cancellation test on route to Board Room:
   - Dispatches navigation
   - Observes robot moving
   - Issues cancellation
   - Verifies stationary stop without overshoot or runaway
5. Clean return to Lobby verifying recovery from cancellation
"""

import json
import math
import sys
import time
from typing import Any, Dict, Optional, Tuple
import urllib.request
import urllib.error

BASE_URL = "http://localhost:8000"


def http_get(path: str) -> Tuple[int, Dict[str, Any]]:
    req = urllib.request.Request(f"{BASE_URL}{path}", headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read().decode())
            return resp.status, data
    except urllib.error.HTTPError as e:
        data = json.loads(e.read().decode()) if e.headers.get_content_type() == "application/json" else {"detail": str(e)}
        return e.code, data
    except Exception as e:
        return 0, {"error": str(e)}


def http_post(path: str, payload: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
    body = json.dumps(payload).encode()
    req = urllib.request.Request(f"{BASE_URL}{path}", data=body, headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
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


def run_e2e_test():
    print("=" * 70)
    print("STAGE B: LIVE GAZEBO INTEGRATION TEST - ROBOT CONTROL API")
    print("=" * 70)

    # 1. Wait for API and Nav2 stack readiness
    print("\n[Step 1] Checking API & Nav2 stack readiness...")
    ready = False
    for i in range(30):
        code, health = http_get("/health")
        if code == 200 and health.get("ros2_connected") and health.get("nav2_action_server_ready"):
            print(f" -> System ready: {health}")
            ready = True
            break
        print(f" -> Waiting for Nav2 stack readiness ({i+1}/30)... status={health.get('overall_status', 'unreachable')}")
        time.sleep(2)

    if not ready:
        print("FAIL: Stack did not become ready in time.")
        sys.exit(1)

    # 2. Verify approved locations
    print("\n[Step 2] Verifying approved locations endpoint...")
    code, loc_data = http_get("/locations")
    assert code == 200, f"Expected 200, got {code}"
    locations = {loc["id"]: loc for loc in loc_data["locations"]}
    print(f" -> Retrieved {len(locations)} approved locations: {list(locations.keys())}")
    assert "lobby" in locations
    assert "living_demo" in locations
    assert "board_room" in locations
    assert "accounts_ws" in locations

    # 3. Verify status endpoint freshness
    print("\n[Step 3] Checking live robot status...")
    code, status = http_get("/status")
    assert code == 200, f"Expected 200, got {code}"
    print(f" -> Pose: {status.get('pose')}")
    print(f" -> Velocity: {status.get('velocity')}")
    print(f" -> LiDAR: {status.get('lidar')}")
    assert status.get("pose") is not None, "Pose should not be null"
    assert status.get("velocity") is not None, "Velocity should not be null"

    # 4. Whitelist rejection test
    print("\n[Step 4] Testing whitelist enforcement with invalid location...")
    code, err_data = http_post("/navigate", {"location_id": "restricted_area"})
    print(f" -> Response code: {code}, detail: {err_data.get('detail')}")
    assert code == 422, f"Expected 422 Unprocessable Entity, got {code}"

    # 5. Live Navigation: Lobby -> Living Demo
    print("\n[Step 5] Dispatching navigation: Lobby -> Living Demo...")
    code, nav_resp = http_post("/navigate", {"location_id": "living_demo"})
    assert code == 202, f"Expected 202 Accepted, got {code}"
    cmd_id = nav_resp["command_id"]
    print(f" -> Accepted command: {cmd_id}")

    # 6. Concurrency lock test while navigating
    print("\n[Step 6] Testing concurrency lock (rejecting concurrent navigation)...")
    time.sleep(1.0)
    code, conflict = http_post("/navigate", {"location_id": "board_room"})
    print(f" -> Concurrency test code: {code}, detail: {conflict.get('detail')}")
    assert code == 409, f"Expected 409 Conflict, got {code}"

    # 7. Monitor Living Demo navigation to completion
    print("\n[Step 7] Monitoring navigation to Living Demo...")
    start_time = time.time()
    nav_succeeded = False
    while time.time() - start_time < 90:
        code, cmd_status = http_get(f"/commands/{cmd_id}")
        assert code == 200
        st = cmd_status["status"]
        dist = cmd_status.get("distance_remaining_m")
        rec = cmd_status.get("recoveries", 0)
        print(f" -> [{time.time() - start_time:.1f}s] Status: {st}, Distance remaining: {dist}m, Recoveries: {rec}")

        if st == "succeeded":
            nav_succeeded = True
            print(" -> Arrived at Living Demo successfully!")
            break
        elif st in ("failed", "canceled"):
            print(f"FAIL: Navigation ended with status '{st}': {cmd_status.get('error_message')}")
            sys.exit(1)
        time.sleep(2)

    assert nav_succeeded, "Navigation timed out after 90 seconds"

    # Verify arrived pose matches Living Demo
    code, status_after = http_get("/status")
    cur_pose = status_after["pose"]
    target = locations["living_demo"]
    dist_error = math.hypot(cur_pose["x"] - target["x"], cur_pose["y"] - target["y"])
    print(f" -> Final pose: ({cur_pose['x']}, {cur_pose['y']}), Target: ({target['x']}, {target['y']})")
    print(f" -> Distance error: {dist_error:.4f} m (threshold: 0.35 m)")
    assert dist_error <= 0.35, f"Final position error {dist_error:.4f}m exceeds tolerance 0.35m"

    # 8. Test Cancellation Flow: Dispatch toward Board Room, then cancel
    print("\n[Step 8] Testing Safe Cancellation on route to Board Room...")
    code, nav_resp2 = http_post("/navigate", {"location_id": "board_room"})
    assert code == 202
    cmd_id2 = nav_resp2["command_id"]
    print(f" -> Dispatched command {cmd_id2} to Board Room")

    # Let the robot accelerate
    print(" -> Waiting for robot to begin moving...")
    moving = False
    for _ in range(15):
        time.sleep(0.5)
        code, st = http_get("/status")
        v = st.get("velocity", {})
        lin_v = abs(v.get("linear_x", 0.0))
        ang_v = abs(v.get("angular_z", 0.0))
        if lin_v > 0.05 or ang_v > 0.05:
            print(f" -> Robot is moving: linear={lin_v:.3f} m/s, angular={ang_v:.3f} rad/s")
            moving = True
            break

    # Issue cancellation
    print(" -> Issuing cancellation request...")
    cancel_t0 = time.time()
    code, cancel_resp = http_post(f"/commands/{cmd_id2}/cancel", {})
    cancel_elapsed = time.time() - cancel_t0
    print(f" -> Cancel response ({cancel_elapsed:.2f}s): code={code}, body={cancel_resp}")
    assert code == 200, f"Expected 200 OK, got {code}"
    assert cancel_resp["status"] == "canceled"
    assert cancel_resp["verified_stopped"] is True

    # Verify robot is truly at rest
    time.sleep(0.5)
    code, st_rest = http_get("/status")
    vel_rest = st_rest.get("velocity", {})
    print(f" -> Verified rest velocity: linear={vel_rest.get('linear_x')} m/s, angular={vel_rest.get('angular_z')} rad/s")
    assert abs(vel_rest.get("linear_x", 0.0)) < 0.05
    assert abs(vel_rest.get("angular_z", 0.0)) < 0.05

    # 9. Clean Return to Lobby
    print("\n[Step 9] Verifying clean return to Lobby after cancellation...")
    code, return_resp = http_post("/navigate", {"location_id": "lobby"})
    assert code == 202, f"Expected 202 Accepted, got {code}"
    return_cmd_id = return_resp["command_id"]
    print(f" -> Return command {return_cmd_id} accepted")

    ret_start = time.time()
    ret_succeeded = False
    while time.time() - ret_start < 90:
        code, cmd_st = http_get(f"/commands/{return_cmd_id}")
        assert code == 200
        st = cmd_st["status"]
        dist = cmd_st.get("distance_remaining_m")
        print(f" -> [{time.time() - ret_start:.1f}s] Status: {st}, Distance remaining: {dist}m")
        if st == "succeeded":
            ret_succeeded = True
            print(" -> Arrived back at Lobby successfully!")
            break
        elif st in ("failed", "canceled"):
            print(f"FAIL: Return navigation ended with status '{st}'")
            sys.exit(1)
        time.sleep(2)

    assert ret_succeeded, "Return to Lobby timed out"

    print("\n" + "=" * 70)
    print("ALL STAGE B LIVE GAZEBO INTEGRATION TESTS PASSED SUCCESSFULLY!")
    print("=" * 70)


if __name__ == "__main__":
    run_e2e_test()

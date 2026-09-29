#!/usr/bin/env python3
"""
Audit Test Script for Requirement 4:
- 4A: Cancellation while the robot is actively rotating
- 4B: Cancellation followed by a new navigation goal
- 4C: API shutdown during an active goal and subsequent clean recovery
"""

import json
import math
import subprocess
import sys
import time
import urllib.request
import urllib.error

BASE_URL = "http://localhost:8000"


def http_get(path: str):
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


def http_post(path: str, payload: dict):
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


def test_audit():
    print("=" * 75)
    print("AUDIT REQUIREMENT 4: CANCELLATION BEHAVIOR & LIFECYCLE AUDIT")
    print("=" * 75)

    code, health = http_get("/health")
    assert code == 200 and health.get("overall_status") == "healthy", f"System not healthy: {health}"
    print(f"[Init] System is healthy: {health}")

    code, st0 = http_get("/status")
    print(f"[Init] Robot pose: {st0.get('pose')}")

    # -------------------------------------------------------------
    # 4A: Cancellation while robot is actively rotating
    # -------------------------------------------------------------
    print("\n--- Test 4A: Cancellation while robot is rotating ---")
    # Dispatch navigation that requires turning:
    # If robot is facing West (yaw ~ 3.14), target East (living_demo or lobby) requires ~180 deg turn
    # If robot is facing North, target West requires 90 deg turn
    code, res = http_post("/navigate", {"location_id": "living_demo"})
    assert code == 202, f"Failed to dispatch: {res}"
    cmd_id = res["command_id"]
    print(f" -> Dispatched command {cmd_id} towards Living Demo")

    # Monitor for active rotation
    print(" -> Polling for active rotation...")
    rotating = False
    rot_speed = 0.0
    for _ in range(40):
        time.sleep(0.15)
        code, st = http_get("/status")
        v = st.get("velocity", {})
        wz = abs(v.get("angular_z", 0.0))
        if wz > 0.08:
            rotating = True
            rot_speed = wz
            print(f" -> Detected active rotation: angular_z = {wz:.3f} rad/s")
            break

    assert rotating, "Robot did not enter rotation phase"

    # Issue cancellation during active rotation
    print(f" -> Issuing cancellation while rotating at {rot_speed:.3f} rad/s...")
    t0 = time.time()
    code, cancel_res = http_post(f"/commands/{cmd_id}/cancel", {})
    t_cancel = time.time() - t0
    print(f" -> Cancellation response in {t_cancel:.2f}s: code={code}, body={cancel_res}")
    assert code == 200, f"Expected 200, got {code}"
    assert cancel_res["status"] == "canceled"
    assert cancel_res["verified_stopped"] is True

    # Check that robot has settled to rest
    time.sleep(0.5)
    code, st = http_get("/status")
    vel = st.get("velocity", {})
    print(f" -> Rest verification: linear_x={vel.get('linear_x')} m/s, angular_z={vel.get('angular_z')} rad/s")
    assert abs(vel.get("linear_x", 0.0)) < 0.03, f"Linear vel not at rest: {vel}"
    assert abs(vel.get("angular_z", 0.0)) < 0.03, f"Angular vel not at rest: {vel}"
    print(" -> [PASS] Test 4A: Cleanly canceled while actively rotating; reached full rest.")

    # -------------------------------------------------------------
    # 4B: Cancellation followed by a new navigation goal
    # -------------------------------------------------------------
    print("\n--- Test 4B: Cancellation followed by new navigation goal ---")
    code, res2 = http_post("/navigate", {"location_id": "living_demo"})
    assert code == 202, f"Failed to dispatch new goal: {res2}"
    cmd_id2 = res2["command_id"]
    print(f" -> Dispatched new goal {cmd_id2} to Living Demo")

    t_start = time.time()
    succeeded = False
    while time.time() - t_start < 60:
        code, cst = http_get(f"/commands/{cmd_id2}")
        st = cst.get("status")
        rem = cst.get("distance_remaining_m")
        if st == "succeeded":
            succeeded = True
            print(f" -> Arrived at Living Demo in {time.time() - t_start:.1f}s!")
            break
        elif st in ("failed", "canceled"):
            print(f"FAIL: Command ended with status '{st}'")
            sys.exit(1)
        time.sleep(1.5)

    assert succeeded, "Navigation to Living Demo timed out"

    code, st = http_get("/status")
    pose = st.get("pose", {})
    target = (-2.50, 10.50)
    err = math.hypot(pose["x"] - target[0], pose["y"] - target[1])
    print(f" -> Final pose: ({pose['x']}, {pose['y']}), Target: {target}, Error: {err:.4f} m")
    assert err <= 0.35, f"Arrival error {err:.4f} m exceeds tolerance"
    print(" -> [PASS] Test 4B: New goal chained immediately after cancel and arrived accurately.")

    # -------------------------------------------------------------
    # 4C: API shutdown during an active goal
    # -------------------------------------------------------------
    print("\n--- Test 4C: API shutdown during active goal ---")
    # Dispatch towards Board Room
    code, res3 = http_post("/navigate", {"location_id": "board_room"})
    assert code == 202
    cmd_id3 = res3["command_id"]
    print(f" -> Dispatched {cmd_id3} towards Board Room")

    # Wait until robot is underway
    print(" -> Waiting for robot to begin transit...")
    underway = False
    for _ in range(20):
        time.sleep(0.3)
        code, st = http_get("/status")
        v = st.get("velocity", {})
        if abs(v.get("linear_x", 0.0)) > 0.1 or abs(v.get("angular_z", 0.0)) > 0.1:
            print(f" -> Transit active: {v}")
            underway = True
            break

    assert underway, "Robot did not begin transit"

    # Terminate the uvicorn process cleanly via SIGTERM inside the container
    print(" -> Sending SIGTERM to uvicorn API process to trigger shutdown lifecycle...")
    subprocess.run(["docker", "exec", "ros2_gazebo_robot", "pkill", "-SIGTERM", "-f", "uvicorn"], check=True)
    time.sleep(2.0)

    # Verify API is stopped
    code, _ = http_get("/health")
    assert code == 0, f"Expected API to be unreachable, got {code}"
    print(" -> Verified API process shut down cleanly.")

    # Restart API server
    print(" -> Restarting API server daemon inside container...")
    subprocess.run([
        "docker", "exec", "-d",
        "-e", "PYTHONPATH=/workspace/gazebo_four_wheel_robot",
        "ros2_gazebo_robot",
        "/ros_entrypoint.sh",
        "python3", "-m", "uvicorn", "api.server:app", "--host", "0.0.0.0", "--port", "8000"
    ], check=True)

    # Wait for API recovery
    print(" -> Waiting for API recovery...")
    recovered = False
    for _ in range(20):
        time.sleep(0.5)
        code, h = http_get("/health")
        if code == 200 and h.get("overall_status") == "healthy":
            print(f" -> API recovered and healthy: {h}")
            recovered = True
            break

    assert recovered, "API did not recover after restart"

    # Verify status and send return command to Lobby
    print(" -> Verifying robot status and dispatching return to Lobby...")
    code, st = http_get("/status")
    assert code == 200
    print(f" -> Current pose after recovery: {st.get('pose')}")

    code, res4 = http_post("/navigate", {"location_id": "lobby"})
    assert code == 202
    cmd_id4 = res4["command_id"]
    print(f" -> Dispatched return command {cmd_id4} to Lobby")

    # Monitor return to Lobby
    t_ret = time.time()
    ret_ok = False
    while time.time() - t_ret < 60:
        code, cst = http_get(f"/commands/{cmd_id4}")
        st = cst.get("status")
        if st == "succeeded":
            ret_ok = True
            print(f" -> Arrived back at Lobby in {time.time() - t_ret:.1f}s!")
            break
        elif st in ("failed", "canceled"):
            print(f"FAIL: Return command ended with '{st}'")
            sys.exit(1)
        time.sleep(1.5)

    assert ret_ok, "Return to Lobby timed out"
    print(" -> [PASS] Test 4C: API shutdown during active goal handled cleanly; full recovery confirmed.")

    print("\n" + "=" * 75)
    print("ALL AUDIT REQUIREMENT 4 TESTS PASSED SUCCESSFULLY!")
    print("=" * 75)


if __name__ == "__main__":
    test_audit()

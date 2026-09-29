#!/usr/bin/env python3
"""
Repeat Leg 3 (Board Room -> Accounts Workstations) 3 times.
Measures:
  - Action result
  - Completion pose and error
  - 1-second settled pose and error
  - Settling drift
  - Recoveries and collisions
"""

import sys
import os
import time
import math
import json
import urllib.request
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

def make_pose_stamped(navigator, x, y, yaw, frame_id="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose

def get_robot_status():
    try:
        with urllib.request.urlopen("http://localhost:8000/status", timeout=1.0) as req:
            return json.loads(req.read().decode("utf-8"))
    except Exception:
        return None

def go_to_board_room(navigator):
    """Navigates the robot back to Board Room (-23.00, 9.50) from anywhere."""
    st = get_robot_status()
    cx = st["pose"]["x"] if st and st.get("pose") else 0.0
    cy = st["pose"]["y"] if st and st.get("pose") else 0.0
    print(f"\n[Reset] Repositioning robot to Board Room from ({cx:.2f}, {cy:.2f})...")

    # If inside Accounts (x < -40.0), exit into aisle first
    poses = []
    if cx < -44.0:
        poses.append(make_pose_stamped(navigator, -44.00, 1.50, 0.0))
        poses.append(make_pose_stamped(navigator, -33.50, 1.80, 0.0))
        poses.append(make_pose_stamped(navigator, -33.50, 7.25, 1.5708))
    elif cx < -30.0:
        poses.append(make_pose_stamped(navigator, -33.50, 7.25, 1.5708))
    elif cx > -10.0:
        poses.append(make_pose_stamped(navigator, -2.00, 7.25, 3.1415))

    poses.append(make_pose_stamped(navigator, -23.00, 7.40, 1.5708))
    poses.append(make_pose_stamped(navigator, -23.00, 9.50, 3.1415))

    navigator.goThroughPoses(poses)
    while not navigator.isTaskComplete():
        time.sleep(0.5)
    time.sleep(2.0)
    st_end = get_robot_status()
    bx = st_end["pose"]["x"] if st_end and st_end.get("pose") else 0.0
    by = st_end["pose"]["y"] if st_end and st_end.get("pose") else 0.0
    print(f"[Reset] Robot positioned at Board Room: ({bx:.3f}, {by:.3f})\n")

def run_single_leg3(navigator, run_num, goal_x=-47.00, goal_y=1.50, goal_yaw=3.1415, tolerance=0.35):
    print("=" * 65)
    print(f" RUN {run_num}: BOARD ROOM -> ACCOUNTS WORKSTATIONS")
    print(f" Target Goal: ({goal_x:.2f}, {goal_y:.2f}, yaw={goal_yaw:.4f}) | Tolerance: {tolerance:.2f}m")
    print("=" * 65)

    waypoints = [
        (-23.00, 7.40, 1.5708),   # Exit Board Room south into corridor
        (-33.50, 7.25, -1.5708),  # Corridor west passage opening
        (-33.50, 1.80, 3.1415),   # Project passageway west
        (-44.00, 1.50, 3.1415),   # Approach accounts doorway
    ]
    poses = [make_pose_stamped(navigator, wx, wy, wyaw) for (wx, wy, wyaw) in waypoints]
    poses.append(make_pose_stamped(navigator, goal_x, goal_y, goal_yaw))

    start_time = time.time()
    navigator.goThroughPoses(poses)

    step = 0
    max_recoveries = 0
    while not navigator.isTaskComplete():
        fb = navigator.getFeedback()
        step += 1
        elapsed = time.time() - start_time
        if fb:
            cx = fb.current_pose.pose.position.x
            cy = fb.current_pose.pose.position.y
            dist_rem = fb.distance_remaining
            recovers = fb.number_of_recoveries
            max_recoveries = max(max_recoveries, recovers)
            if step % 8 == 0 or dist_rem < 1.0:
                print(f"  [{elapsed:5.1f}s] Pose: ({cx:6.2f}, {cy:6.2f}) | Rem: {dist_rem:5.2f}m | Rec: {recovers}")
        time.sleep(0.1)

    completion_time = time.time() - start_time
    action_result = navigator.getResult()

    # Capture EXACT pose at completion
    st_comp = get_robot_status()
    pose_comp = st_comp["pose"] if st_comp else {}
    vel_comp = st_comp["velocity"] if st_comp else {}
    cx_comp = pose_comp.get("x", 0.0)
    cy_comp = pose_comp.get("y", 0.0)
    cyaw_comp = pose_comp.get("yaw", 0.0)
    err_comp = math.hypot(cx_comp - goal_x, cy_comp - goal_y)

    # Mechanical settling 1.0s
    time.sleep(1.0)

    # Capture Settled Pose 1 second later
    st_settled = get_robot_status()
    pose_settled = st_settled["pose"] if st_settled else {}
    vel_settled = st_settled["velocity"] if st_settled else {}
    cx_settled = pose_settled.get("x", 0.0)
    cy_settled = pose_settled.get("y", 0.0)
    cyaw_settled = pose_settled.get("yaw", 0.0)
    err_settled = math.hypot(cx_settled - goal_x, cy_settled - goal_y)
    drift = math.hypot(cx_settled - cx_comp, cy_settled - cy_comp)

    success = (action_result == TaskResult.SUCCEEDED) and (err_settled <= tolerance)

    print("-" * 65)
    print(f" Run {run_num} Result Summary:")
    print(f"  - Action Result       : {action_result}")
    print(f"  - Elapsed Time        : {completion_time:.2f}s")
    print(f"  - Pose at Completion  : ({cx_comp:.4f}, {cy_comp:.4f}) | Error: {err_comp:.4f}m")
    print(f"  - Settled Pose (1s)   : ({cx_settled:.4f}, {cy_settled:.4f}) | Error: {err_settled:.4f}m")
    print(f"  - Settling Drift      : {drift:.4f}m")
    print(f"  - Recoveries          : {max_recoveries}")
    print(f"  - Status vs 0.35m Tol : {'PASSED [OK]' if success else 'FAILED [X]'}")
    print("-" * 65)

    return {
        "run": run_num,
        "success": success,
        "action_result": str(action_result),
        "elapsed_s": round(completion_time, 2),
        "pose_at_completion": [round(cx_comp, 4), round(cy_comp, 4), round(cyaw_comp, 4)],
        "error_at_completion_m": round(err_comp, 4),
        "pose_settled_1s_later": [round(cx_settled, 4), round(cy_settled, 4), round(cyaw_settled, 4)],
        "error_settled_m": round(err_settled, 4),
        "settling_drift_m": round(drift, 4),
        "recoveries": max_recoveries
    }

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer="robot_localization")

    results = []
    for run in range(1, 4):
        go_to_board_room(navigator)
        res = run_single_leg3(navigator, run, goal_x=-47.00, goal_y=1.50)
        results.append(res)

    print("\n" + "=" * 65)
    print(" THREE-RUN REPETITION SUMMARY (Board Room -> Accounts WS):")
    print("=" * 65)
    for r in results:
        status = "PASSED [OK]" if r["success"] else "FAILED [X]"
        print(f" Run {r['run']}: {status} | Time: {r['elapsed_s']}s | Settled Error: {r['error_settled_m']}m | Rec: {r['recoveries']}")
    print("=" * 65)

    with open("/workspace/gazebo_four_wheel_robot/leg3_repetition_report.json", "w") as f:
        json.dump(results, f, indent=2)

if __name__ == "__main__":
    main()

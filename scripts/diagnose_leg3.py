#!/usr/bin/env python3
"""
Diagnostic Script for Board Room -> Accounts Workstations Leg
Records:
  - Nav2 action result
  - Goal coordinates
  - Robot pose at action completion
  - Robot settled pose 1 second later
  - Physical position error vs 0.35m tolerance
  - Recoveries and obstacle clearance
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
    """Queries http://localhost:8000/status for live pose and velocity."""
    try:
        with urllib.request.urlopen("http://localhost:8000/status", timeout=1.0) as req:
            return json.loads(req.read().decode("utf-8"))
    except Exception as e:
        return None

def main():
    print("==========================================================")
    print(" DIAGNOSTIC: Board Room -> Accounts Workstations (Leg 3)")
    print("==========================================================")
    rclpy.init()
    navigator = BasicNavigator()
    print("Waiting for Nav2 stack...")
    navigator.waitUntilNav2Active(localizer="robot_localization")
    print("Nav2 is ready!\n")

    # 1. Check current robot position
    status = get_robot_status()
    curr_x = status["pose"]["x"] if status and status.get("pose") else -1.80
    curr_y = status["pose"]["y"] if status and status.get("pose") else 0.00
    print(f"Current Robot Pose: ({curr_x:.3f}, {curr_y:.3f})")

    # If robot is near Lobby (x > -10.0), navigate to Board Room first
    dist_to_board = math.hypot(curr_x - (-23.00), curr_y - 9.50)
    if dist_to_board > 3.0:
        print("\n[Prep] Navigating robot to Board Room starting position (-23.00, 9.50)...")
        prep_poses = [
            make_pose_stamped(navigator, -2.00, 7.25, 3.1415),   # North corridor above lobby
            make_pose_stamped(navigator, -23.00, 7.40, 1.5708),  # Corridor below board room
            make_pose_stamped(navigator, -23.00, 9.50, 3.1415),  # Inside board room
        ]
        navigator.goThroughPoses(prep_poses)
        while not navigator.isTaskComplete():
            time.sleep(0.5)
        print("Robot arrived at Board Room!\n")
        time.sleep(2.0)

    # Re-verify position at Board Room
    status_at_start = get_robot_status()
    start_x = status_at_start["pose"]["x"]
    start_y = status_at_start["pose"]["y"]
    print(f"Starting Pose for Leg 3: ({start_x:.3f}, {start_y:.3f})")

    # Target: Accounts Workstation (nominal coordinate in 669118f: x=-47.00, y=1.50)
    goal_x = -47.00
    goal_y = 1.50
    goal_yaw = 3.1415
    tolerance = 0.35

    print(f"\nCommanding Leg 3 to Accounts Workstations:")
    print(f"  Goal Coordinates: ({goal_x:.2f}, {goal_y:.2f}, yaw={goal_yaw:.4f})")
    print(f"  Nav2 Configured Tolerance: {tolerance:.2f}m")

    # Waypoints as defined in 669118f
    waypoints = [
        (-23.00, 7.40, 1.5708),   # Exit Board Room south into corridor
        (-33.50, 7.25, -1.5708),  # Corridor west passage opening
        (-33.50, 1.80, 3.1415),   # Project passageway west
        (-44.00, 1.50, 3.1415),   # Approach accounts doorway
    ]
    poses = [make_pose_stamped(navigator, wx, wy, wyaw) for (wx, wy, wyaw) in waypoints]
    poses.append(make_pose_stamped(navigator, goal_x, goal_y, goal_yaw))

    print(f"  Using NavigateThroughPoses with {len(poses)} poses...")
    start_time = time.time()
    navigator.goThroughPoses(poses)

    # Monitor until completion
    step = 0
    max_recoveries = 0
    min_dist_rem = 999.0
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
            min_dist_rem = min(min_dist_rem, dist_rem)
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

    print("\n" + "=" * 65)
    print(" MEASUREMENTS AT ACTION COMPLETION:")
    print("=" * 65)
    print(f"  Nav2 Action Result Code : {action_result} (SUCCEEDED={TaskResult.SUCCEEDED})")
    print(f"  Action Elapsed Time     : {completion_time:.2f}s")
    print(f"  Pose at Completion      : x={cx_comp:.4f}, y={cy_comp:.4f}, yaw={cyaw_comp:.4f}")
    print(f"  Velocity at Completion  : lin_x={vel_comp.get('linear_x', 0.0):.4f}, ang_z={vel_comp.get('angular_z', 0.0):.4f}")
    print(f"  Position Error at Comp  : {err_comp:.4f} m (Tolerance: {tolerance:.2f} m)")
    print(f"  Tolerance Satisfied?    : {'YES' if err_comp <= tolerance else 'NO'}")

    # Sleep exactly 1.0 second
    print("\nWaiting 1.0 second for mechanical settling...")
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

    print("=" * 65)
    print(" MEASUREMENTS 1.0 SECOND LATER (SETTLED POSE):")
    print("=" * 65)
    print(f"  Settled Pose            : x={cx_settled:.4f}, y={cy_settled:.4f}, yaw={cyaw_settled:.4f}")
    print(f"  Settled Velocity        : lin_x={vel_settled.get('linear_x', 0.0):.4f}, ang_z={vel_settled.get('angular_z', 0.0):.4f}")
    print(f"  Settling Drift (1s)     : {drift:.4f} m")
    print(f"  Final Position Error    : {err_settled:.4f} m (Tolerance: {tolerance:.2f} m)")
    print(f"  Tolerance Satisfied?    : {'YES' if err_settled <= tolerance else 'NO'}")
    print(f"  Total Recoveries        : {max_recoveries}")
    print("=" * 65)

    res_summary = {
        "action_result": str(action_result),
        "goal_coordinates": [goal_x, goal_y, goal_yaw],
        "tolerance_m": tolerance,
        "elapsed_s": round(completion_time, 2),
        "pose_at_completion": [round(cx_comp, 4), round(cy_comp, 4), round(cyaw_comp, 4)],
        "velocity_at_completion": vel_comp,
        "error_at_completion_m": round(err_comp, 4),
        "pose_settled_1s_later": [round(cx_settled, 4), round(cy_settled, 4), round(cyaw_settled, 4)],
        "velocity_settled_1s_later": vel_settled,
        "error_settled_m": round(err_settled, 4),
        "settling_drift_m": round(drift, 4),
        "recoveries": max_recoveries,
        "tolerance_satisfied": err_settled <= tolerance
    }
    with open("/workspace/gazebo_four_wheel_robot/diagnostics_leg3_report.json", "w") as f:
        json.dump(res_summary, f, indent=2)
    print(f"Report written to /workspace/gazebo_four_wheel_robot/diagnostics_leg3_report.json")

if __name__ == "__main__":
    main()

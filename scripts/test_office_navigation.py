#!/usr/bin/env python3
"""
Office Navigation Test Script
Executes autonomous navigation between Lobby and key destinations:
  - Experience Centres (Living Demo Room)
  - Board Room
  - Accounts Workstations
  - Return to Lobby

Uses NavigateThroughPoses with intermediate corridor waypoints for legs that
cross room doorways, preventing the robot from oscillating at the doorway
threshold when the local costmap inflation overlaps with the room boundary.
"""

import os
import sys
import time
import math
import json
import argparse
import urllib.request
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# Key Destination Coordinates (Map Frame)
DESTINATIONS = {
    "lobby": {
        "name": "Lobby / Reception IN Lane",
        "x": -1.80,
        "y": 0.00,
        "yaw": 1.5708,
        "desc": "Main entrance security turnstile lane facing corridor"
    },
    "living_demo": {
        "name": "Experience Centre: Living Demo Room",
        "x": -2.50,
        "y": 10.50,
        "yaw": 1.5708,
        "desc": "Smart living demonstration lounge"
    },
    "hotel_exp": {
        "name": "Experience Centre: Hotel Room",
        "x": -8.00,
        "y": 10.00,
        "yaw": 1.5708,
        "desc": "Hotel guestroom automation suite"
    },
    "hospital_exp": {
        "name": "Experience Centre: Hospital Room",
        "x": -7.50,
        "y": 3.50,
        "yaw": 0.00,
        "desc": "Healthcare and patient room experience"
    },
    "board_room": {
        "name": "Board Room",
        "x": -23.00,
        "y": 9.50,
        "yaw": 3.1415,
        "desc": "Executive 16-chair conference room and 85\" display"
    },
    "accounts_ws": {
        "name": "Accounts Workstation",
        "x": -47.00,
        "y": 1.50,
        "yaw": 3.1415,
        "desc": "Finance department workstations AW1-AW8 & AD1-AD2"
    }
}

# Corridor waypoints used to exit rooms before long traversals.
# Each is a (x, y, yaw) tuple in map frame.
CORRIDOR_STEP_SOUTH_OF_LIVING_DEMO = (-2.50, 7.40, -1.5708) # Straight south exit from Living Demo
CORRIDOR_CENTRE_BELOW_BOARD_ROOM   = (-23.00, 7.40, 1.5708) # Corridor center, south of Board Room door
CORRIDOR_WEST_PASSAGE_ENTRY        = (-33.50, 7.25, -1.5708) # South corridor opening (X in [-35, -32])
PROJECT_PASSAGEWAY_WEST            = (-33.50, 1.80, 3.1415)  # Westbound passage towards Accounts
CORRIDOR_CENTRE_ABOVE_LOBBY        = (-2.00,  7.25, -1.5708) # Corridor center, right above Lobby opening (balanced clearance)
CORRIDOR_WEST_APPROACH_ACCOUNTS    = (-44.00, 1.50, 3.1415)  # Align heading directly west towards Accounts doorway

# Tour leg definitions:
TOUR_LEGS = [
    {
        "leg_id": 1,
        "name": "Leg 1: Lobby -> Experience Centre (Living Demo)",
        "timeout": 90.0,
        "waypoints": [],
        "final_dest": "living_demo"
    },
    {
        "leg_id": 2,
        "name": "Leg 2: Experience Centre -> Board Room",
        "timeout": 120.0,
        "waypoints": [
            CORRIDOR_STEP_SOUTH_OF_LIVING_DEMO,  # Exit straight south into corridor (Y=7.40)
            CORRIDOR_CENTRE_BELOW_BOARD_ROOM,    # Travel west along corridor centerline
        ],
        "final_dest": "board_room"
    },
    {
        "leg_id": 3,
        "name": "Leg 3: Board Room -> Accounts Workstations",
        "timeout": 150.0,
        "waypoints": [
            CORRIDOR_CENTRE_BELOW_BOARD_ROOM,    # Exit Board Room south into corridor
            CORRIDOR_WEST_PASSAGE_ENTRY,         # Travel west along corridor to passage opening
            PROJECT_PASSAGEWAY_WEST,             # Turn south through passage into west aisle
            CORRIDOR_WEST_APPROACH_ACCOUNTS,     # Align heading directly west towards Accounts doorway
        ],
        "final_dest": "accounts_ws"
    },
    {
        "leg_id": 4,
        "name": "Leg 4: Accounts Workstations -> Return to Lobby",
        "timeout": 250.0,
        "waypoints": [
            PROJECT_PASSAGEWAY_WEST,             # Travel east through west aisle
            CORRIDOR_WEST_PASSAGE_ENTRY,         # Turn north back into main corridor
            CORRIDOR_CENTRE_ABOVE_LOBBY,         # Travel east along main corridor
        ],
        "final_dest": "lobby"
    }
]


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


def wait_for_completion(navigator, timeout, leg_name, step_label=""):
    """Poll until task is complete or timeout. Returns (success, elapsed, trajectory, metrics)."""
    start_time = time.time()
    trajectory = []
    step = 0
    max_linear_vel = 0.0
    sum_moving_vel = 0.0
    moving_count = 0
    min_lidar = 999.0
    final_recoveries = 0

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        step += 1
        elapsed = time.time() - start_time

        cur_vel = 0.0
        try:
            with urllib.request.urlopen("http://localhost:8000/status", timeout=0.5) as req:
                sdata = json.loads(req.read().decode("utf-8"))
                vel = sdata.get("velocity", {})
                lidar = sdata.get("lidar", {})
                cur_vel = abs(vel.get("linear_x", 0.0))
                cur_lidar = lidar.get("min_lidar_m", 999.0)
                if cur_vel > max_linear_vel:
                    max_linear_vel = cur_vel
                if cur_vel > 0.05:
                    sum_moving_vel += cur_vel
                    moving_count += 1
                if cur_lidar < min_lidar:
                    min_lidar = cur_lidar
        except Exception:
            pass

        if feedback:
            cx = feedback.current_pose.pose.position.x
            cy = feedback.current_pose.pose.position.y
            dist_rem = feedback.distance_remaining
            recovers = feedback.number_of_recoveries
            final_recoveries = max(final_recoveries, recovers)
            trajectory.append({
                "time": round(elapsed, 2),
                "x": round(cx, 3),
                "y": round(cy, 3),
                "dist_rem": round(dist_rem, 3),
                "vel": round(cur_vel, 3)
            })

            if step % 4 == 0 or dist_rem < 0.8:
                label = f" [{step_label}]" if step_label else ""
                print(f"  [{elapsed:5.1f}s]{label} Pose: ({cx:6.2f}, {cy:6.2f}) | Vel: {cur_vel:4.2f}m/s | Rem: {dist_rem:5.2f}m | Rec: {recovers}")

        if elapsed > timeout:
            print(f"  [TIMEOUT] Exceeded {timeout}s — cancelling...")
            navigator.cancelTask()
            while not navigator.isTaskComplete():
                time.sleep(0.1)
            metrics = {
                "peak_vel": round(max_linear_vel, 3),
                "avg_vel": round(sum_moving_vel / max(1, moving_count), 3),
                "recoveries": final_recoveries,
                "min_lidar": round(min_lidar, 3) if min_lidar < 900 else None
            }
            return False, elapsed, trajectory, metrics

        time.sleep(0.3)

    elapsed = time.time() - start_time
    result = navigator.getResult()
    success = (result == TaskResult.SUCCEEDED)
    metrics = {
        "peak_vel": round(max_linear_vel, 3),
        "avg_vel": round(sum_moving_vel / max(1, moving_count), 3),
        "recoveries": final_recoveries,
        "min_lidar": round(min_lidar, 3) if min_lidar < 900 else None
    }
    return success, elapsed, trajectory, metrics


def get_settled_pose(timeout=4.0):
    """Queries http://localhost:8000/status for the settled robot pose once stopped."""
    start = time.time()
    last_pose = None
    while time.time() - start < timeout:
        try:
            with urllib.request.urlopen("http://localhost:8000/status", timeout=1.0) as req:
                data = json.loads(req.read().decode("utf-8"))
                pose = data.get("pose")
                vel = data.get("velocity")
                if pose:
                    last_pose = (pose.get("x"), pose.get("y"), pose.get("yaw"))
                if pose and vel:
                    lin_v = abs(vel.get("linear_x", 1.0))
                    ang_v = abs(vel.get("angular_z", 1.0))
                    if lin_v < 0.05 and ang_v < 0.05:
                        return last_pose
        except Exception:
            pass
        time.sleep(0.2)
    if last_pose:
        return last_pose
    return None, None, None


def navigate_leg(navigator, leg):
    dest = DESTINATIONS[leg["final_dest"]]
    gx, gy, gyaw = dest["x"], dest["y"], dest["yaw"]

    print("\n" + "=" * 65)
    print(f" {leg['name'].upper()}")
    print(f" Target: {dest['name']} ({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f} rad)")
    print(f" Description: {dest['desc']}")
    if leg["waypoints"]:
        print(f" Via {len(leg['waypoints'])} intermediate corridor waypoint(s)")
    print("=" * 65)

    waypoints = leg["waypoints"]
    timeout = leg["timeout"]

    all_trajectory = []
    start_time = time.time()

    if waypoints:
        # Build pose list: waypoints + final goal
        poses = []
        for (wx, wy, wyaw) in waypoints:
            poses.append(make_pose_stamped(navigator, wx, wy, wyaw))
        poses.append(make_pose_stamped(navigator, gx, gy, gyaw))

        print(f"  Using NavigateThroughPoses with {len(poses)} pose(s)...")
        navigator.goThroughPoses(poses)
    else:
        goal_pose = make_pose_stamped(navigator, gx, gy, gyaw)
        navigator.goToPose(goal_pose)

    success, elapsed, trajectory, metrics = wait_for_completion(
        navigator, timeout, leg["name"])
    all_trajectory.extend(trajectory)

    print("-" * 65)
    settled_x, settled_y, _ = get_settled_pose()
    if settled_x is not None:
        final_x, final_y = settled_x, settled_y
    else:
        final_x = all_trajectory[-1]["x"] if all_trajectory else None
        final_y = all_trajectory[-1]["y"] if all_trajectory else None

    pos_err = math.hypot(final_x - gx, final_y - gy) if final_x is not None else None
    total_elapsed = time.time() - start_time
    MAX_ALLOWED_ERROR_M = 0.35  # Strict check against Nav2 xy_goal_tolerance (0.35m)
    physical_arrival_ok = (pos_err is not None and pos_err <= MAX_ALLOWED_ERROR_M)

    print(f"  Speed: Peak = {metrics['peak_vel']:.2f} m/s | Avg = {metrics['avg_vel']:.2f} m/s | Rec = {metrics['recoveries']} | Min LiDAR = {metrics['min_lidar']} m")

    if success and physical_arrival_ok:
        print(f"  RESULT: SUCCESS - Goal Reached in {total_elapsed:.1f}s!")
        print(f"  Final Settled Position: ({final_x:.3f}, {final_y:.3f}) | Error: {pos_err:.3f}m (tolerance: {MAX_ALLOWED_ERROR_M}m)")
        return {
            "success": True,
            "target": dest["name"],
            "elapsed": round(total_elapsed, 2),
            "final_pose": (round(final_x, 3), round(final_y, 3)),
            "error_m": round(pos_err, 3),
            "peak_vel": metrics["peak_vel"],
            "avg_vel": metrics["avg_vel"],
            "recoveries": metrics["recoveries"],
            "min_lidar": metrics["min_lidar"],
            "trajectory_points": len(all_trajectory),
            "trajectory": all_trajectory
        }
    else:
        if success and not physical_arrival_ok:
            reason = f"GOAL_ERROR_EXCEEDED ({pos_err:.3f}m > {MAX_ALLOWED_ERROR_M}m)"
        elif not success and total_elapsed >= timeout:
            reason = "TIMEOUT"
        else:
            reason = "FAILED"
        print(f"  RESULT: {reason} after {total_elapsed:.1f}s")
        if final_x is not None:
            print(f"  Final Settled Position: ({final_x:.3f}, {final_y:.3f}) | Error: {pos_err:.3f}m (tolerance: {MAX_ALLOWED_ERROR_M}m)")
        return {
            "success": False,
            "reason": reason,
            "elapsed": round(total_elapsed, 2),
            "final_pose": (round(final_x, 3), round(final_y, 3)) if final_x is not None else None,
            "error_m": round(pos_err, 3) if pos_err is not None else None,
            "peak_vel": metrics["peak_vel"],
            "avg_vel": metrics["avg_vel"],
            "recoveries": metrics["recoveries"],
            "min_lidar": metrics["min_lidar"],
            "trajectory": all_trajectory
        }


def main():
    parser = argparse.ArgumentParser(description="Test Office Autonomous Navigation with Nav2")
    parser.add_argument("--leg", type=str, default="demo",
                        help="Leg to run: demo (short demo: Lobby -> Experience Centre -> Board Room), all (full tour), or 1, 2, 3, 4")
    parser.add_argument("--report", type=str,
                        default="/workspace/gazebo_four_wheel_robot/office_nav_report.json",
                        help="Path to output JSON report")
    args = parser.parse_args()

    print("==========================================================")
    print(" Office Autonomous Navigation Suite")
    print(" Stack: ROS 2 Jazzy + Gazebo Harmonic + Nav2")
    print(" Map: Casadigi Office Floor Plan (1720x620, 0.05m/px)")
    print("==========================================================")

    rclpy.init()
    navigator = BasicNavigator()
    print("Waiting for Nav2 stack to be active and ready...")
    navigator.waitUntilNav2Active(localizer="robot_localization")
    print("Nav2 is active and ready for navigation!\n")

    results = []

    if args.leg == "all":
        legs_to_run = TOUR_LEGS
    elif args.leg in ("demo", "short"):
        legs_to_run = [l for l in TOUR_LEGS if l["leg_id"] in (1, 2)]
    elif args.leg.isdigit():
        idx = int(args.leg)
        legs_to_run = [l for l in TOUR_LEGS if l["leg_id"] == idx]
        if not legs_to_run:
            print(f"No leg with id {idx}")
            sys.exit(1)
    else:
        print(f"Unknown leg: {args.leg}. Use demo, all, or 1, 2, 3, 4.")
        sys.exit(1)

    total_start = time.time()
    for leg in legs_to_run:
        res = navigate_leg(navigator, leg)
        results.append({
            "leg_id": leg["leg_id"],
            "leg_name": leg["name"],
            "result": res
        })
        if not res["success"]:
            print(f"\n[ABORT] Tour interrupted due to failure in {leg['name']}")
            break
        time.sleep(2.0)

    total_elapsed = time.time() - total_start
    print("\n" + "=" * 65)
    print(" TOUR SUMMARY")
    print(f" Total time: {total_elapsed:.1f}s")
    for r in results:
        status = "PASSED [OK]" if r["result"]["success"] else "FAILED [X]"
        t = r["result"]["elapsed"]
        err = r["result"].get("error_m", "N/Am")
        pv = r["result"].get("peak_vel", "N/A")
        rec = r["result"].get("recoveries", 0)
        print(f" - {r['leg_name']:46s}: {status} ({t}s | err: {err}m | peak: {pv}m/s | rec: {rec})")
    print("=" * 65)

    os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
    with open(args.report, "w") as f:
        json.dump({
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "total_elapsed_s": round(total_elapsed, 2),
            "legs": results
        }, f, indent=2)
    print(f"Report written to: {args.report}")

    all_succeeded = all(r["result"]["success"] for r in results)
    sys.exit(0 if all_succeeded else 1)


if __name__ == "__main__":
    main()

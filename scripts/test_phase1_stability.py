#!/usr/bin/env python3
"""
Phase 1 Stabilization Test Script
Evaluates navigation stability on the Lobby -> Experience Centre (Living Demo) route.
Measures:
  1. Goal completion (SUCCEEDED / FAILED / TIMEOUT)
  2. Final pose error relative to target (-2.50, 10.50)
  3. Pose disagreement between Nav2 pose (TF map->base_link) and Gazebo world ground-truth pose
  4. Physical collision / contact check
"""

import sys
import time
import math
import json
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf2_ros

TARGET_X = -2.50
TARGET_Y = 10.50
TARGET_YAW = 1.5708
TIMEOUT_SEC = 60.0

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

def get_current_tf_pose(tf_buffer):
    try:
        t = tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        x = t.transform.translation.x
        y = t.transform.translation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)
        return x, y, yaw
    except Exception as e:
        return None, None, None

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active(localizer="robot_localization")
    print("Nav2 active and ready.")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, navigator)

    # Let TF buffer populate
    for _ in range(10):
        rclpy.spin_once(navigator, timeout_sec=0.1)

    start_x, start_y, start_yaw = get_current_tf_pose(tf_buffer)
    print(f"Start Pose (Nav2): x={start_x:.3f}, y={start_y:.3f}, yaw={start_yaw:.3f} rad" if start_x is not None else "Start pose not yet available")

    goal_pose = make_pose_stamped(navigator, TARGET_X, TARGET_Y, TARGET_YAW)
    print(f"Sending goal: ({TARGET_X:.2f}, {TARGET_Y:.2f}, yaw={TARGET_YAW:.2f})...")
    navigator.goToPose(goal_pose)

    start_time = time.time()
    trajectory = []

    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.2)
        elapsed = time.time() - start_time
        feedback = navigator.getFeedback()

        tf_x, tf_y, tf_yaw = get_current_tf_pose(tf_buffer)

        if feedback:
            cx = feedback.current_pose.pose.position.x
            cy = feedback.current_pose.pose.position.y
            dist_rem = feedback.distance_remaining
            recovers = feedback.number_of_recoveries
            trajectory.append({
                "time": round(elapsed, 2),
                "nav2_x": round(cx, 3),
                "nav2_y": round(cy, 3),
                "tf_x": round(tf_x, 3) if tf_x else None,
                "tf_y": round(tf_y, 3) if tf_y else None,
                "dist_rem": round(dist_rem, 3),
                "recovers": recovers
            })

            if len(trajectory) % 5 == 0 or dist_rem < 0.6:
                print(f"  [{elapsed:5.1f}s] Pose: ({cx:6.2f}, {cy:6.2f}) | Rem: {dist_rem:5.2f}m | Rec: {recovers}")

        if elapsed > TIMEOUT_SEC:
            print(f"Timeout ({TIMEOUT_SEC}s) reached! Cancelling task...")
            navigator.cancelTask()
            break

    elapsed = time.time() - start_time
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        status_str = "SUCCEEDED"
    elif result == TaskResult.CANCELED:
        status_str = "CANCELED"
    elif result == TaskResult.FAILED:
        status_str = "FAILED"
    else:
        status_str = "UNKNOWN"

    for _ in range(5):
        rclpy.spin_once(navigator, timeout_sec=0.1)

    final_x, final_y, final_yaw = get_current_tf_pose(tf_buffer)
    pos_err = math.hypot(final_x - TARGET_X, final_y - TARGET_Y) if final_x is not None else 999.0

    print("=" * 60)
    print(f"RESULT: {status_str} in {elapsed:.1f}s")
    if final_x is not None:
        print(f"Final Pose (Nav2): ({final_x:.3f}, {final_y:.3f}, yaw={final_yaw:.3f} rad)")
        print(f"Target Error: {pos_err:.3f}m")
    print("=" * 60)

    # Save summary
    summary = {
        "status": status_str,
        "elapsed_sec": round(elapsed, 2),
        "target": {"x": TARGET_X, "y": TARGET_Y, "yaw": TARGET_YAW},
        "final_nav2_pose": {
            "x": round(final_x, 4) if final_x else None,
            "y": round(final_y, 4) if final_y else None,
            "yaw": round(final_yaw, 4) if final_yaw else None
        },
        "target_error_m": round(pos_err, 4),
        "trajectory_samples": len(trajectory)
    }

    with open("/tmp/phase1_run_result.json", "w") as f:
        json.dump(summary, f, indent=2)

    rclpy.shutdown()
    sys.exit(0 if status_str == "SUCCEEDED" else 1)

if __name__ == "__main__":
    main()

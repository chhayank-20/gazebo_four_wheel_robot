#!/usr/bin/env python3
"""
Cancellation Braking Test Script
Starts navigation towards Board Room, waits until robot reaches high forward speed (~1.0 m/s),
triggers cancellation, and samples velocity, stopping distance, and deceleration at 20Hz.
"""

import sys
import time
import math
import json
import urllib.request
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

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

def get_status():
    try:
        with urllib.request.urlopen("http://localhost:8000/status", timeout=0.5) as req:
            return json.loads(req.read().decode("utf-8"))
    except Exception:
        return {}

def main():
    rclpy.init()
    navigator = BasicNavigator()
    print("Waiting for Nav2 to be ready...")
    navigator.waitUntilNav2Active(localizer="robot_localization")
    print("Nav2 is ready.")

    # Target: Board Room via corridor
    # From Living Demo (-2.38, 10.26) to corridor waypoint (-2.5, 7.4) and then (-23.0, 7.4)
    poses = [
        make_pose_stamped(navigator, -2.50, 7.40, -1.5708),
        make_pose_stamped(navigator, -23.00, 7.40, 1.5708),
        make_pose_stamped(navigator, -23.00, 9.50, 3.1415),
    ]

    print("Dispatching navigation towards Board Room...")
    navigator.goThroughPoses(poses)

    print("Monitoring velocity until speed >= 0.85 m/s...")
    start_time = time.time()
    top_speed_reached = 0.0
    cancel_sent = False
    samples = []

    # Monitor loop
    while time.time() - start_time < 30.0:
        st = get_status()
        vel = st.get("velocity", {})
        lin_v = abs(vel.get("linear_x", 0.0))
        pose = st.get("pose", {})
        px, py = pose.get("x", 0.0), pose.get("y", 0.0)

        if lin_v > top_speed_reached:
            top_speed_reached = lin_v

        print(f"  [T+{time.time()-start_time:.1f}s] Pose: ({px:6.2f}, {py:6.2f}) | Vel: {lin_v:.2f} m/s")

        if lin_v >= 0.85 and not cancel_sent:
            print(f"\n>>> TARGET SPEED DETECTED ({lin_v:.2f} m/s)! TRIGGERING CANCELLATION NOW <<<")
            t_cancel = time.time()
            cancel_pose = (px, py)
            cancel_speed = lin_v
            navigator.cancelTask()
            cancel_sent = True
            break

        time.sleep(0.15)

    if not cancel_sent:
        print("[FAIL] Did not reach threshold speed within timeout.")
        navigator.cancelTask()
        sys.exit(1)

    # Now record stopping dynamics at 20Hz (every 50ms)
    print("Recording stopping deceleration dynamics...")
    stop_start = time.time()
    halted = False
    while time.time() - stop_start < 5.0:
        t_sample = time.time() - stop_start
        st = get_status()
        vel = st.get("velocity", {})
        lin_v = abs(vel.get("linear_x", 0.0))
        ang_v = abs(vel.get("angular_z", 0.0))
        pose = st.get("pose", {})
        px, py = pose.get("x", 0.0), pose.get("y", 0.0)
        dist_from_cancel = math.hypot(px - cancel_pose[0], py - cancel_pose[1])
        min_lidar = st.get("lidar", {}).get("min_lidar_m", 999.0)

        samples.append({
            "dt": round(t_sample, 3),
            "lin_v": round(lin_v, 3),
            "ang_v": round(ang_v, 3),
            "dist": round(dist_from_cancel, 3),
            "min_lidar": round(min_lidar, 3)
        })

        if lin_v < 0.02 and ang_v < 0.02:
            halted = True
            break
        time.sleep(0.05)

    # Let robot settle completely
    time.sleep(0.5)
    final_st = get_status()
    f_pose = final_st.get("pose", {})
    fx, fy = f_pose.get("x", 0.0), f_pose.get("y", 0.0)
    stop_distance = math.hypot(fx - cancel_pose[0], fy - cancel_pose[1])
    stopping_time = samples[-1]["dt"] if samples else 0.0
    avg_decel = cancel_speed / max(0.001, stopping_time)

    print("\n" + "=" * 55)
    print(" CANCELLATION BRAKING RESULTS")
    print("=" * 55)
    print(f" Pre-cancel Velocity : {cancel_speed:.3f} m/s")
    print(f" Stopping Time       : {stopping_time:.3f} s")
    print(f" Stopping Distance   : {stop_distance:.3f} m")
    print(f" Average Deceleration: {avg_decel:.3f} m/s^2")
    print(f" Final Velocity      : {final_st.get('velocity', {}).get('linear_x', 0.0):.3f} m/s")
    print(f" Final Pose          : ({fx:.3f}, {fy:.3f})")
    print(f" Min LiDAR Clearance : {final_st.get('lidar', {}).get('min_lidar_m', 0.0):.3f} m")
    print(" Deceleration trace (first 8 samples):")
    for s in samples[:8]:
        print(f"   +dt={s['dt']:4.2f}s | lin_v={s['lin_v']:4.2f} m/s | stop_dist={s['dist']:4.2f} m")
    print("=" * 55)

    passed = halted and stop_distance < 1.0 and stopping_time < 3.0
    print(f"BRAKING TEST: {'PASSED [OK]' if passed else 'FAILED [X]'}\n")
    sys.exit(0 if passed else 1)

if __name__ == "__main__":
    main()

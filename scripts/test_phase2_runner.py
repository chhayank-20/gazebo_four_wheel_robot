#!/usr/bin/env python3
"""
Phase 2 Full Route Runner
Executes one full sequence of 4 legs:
  Leg 1: Lobby -> Living Demo
  Leg 2: Living Demo -> Board Room
  Leg 3: Board Room -> Accounts
  Leg 4: Accounts -> Lobby

Logs detailed metrics:
  - Nav2 pose & Gazebo ground-truth pose
  - Position disagreement (m)
  - Distance from requested goal to physical pose (m)
  - Minimum LiDAR range encountered (m)
  - Commanded velocities (max linear/angular)
  - Recoveries count
  - Physical wall contacts / wedging check
"""

import sys
import os
import time
import math
import json
import argparse
import subprocess
import re
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import tf2_ros

DESTINATIONS = {
    "lobby": {
        "name": "Lobby / Reception",
        "x": -1.80,
        "y": 0.00,
        "yaw": 1.5708,
    },
    "living_demo": {
        "name": "Experience Centre: Living Demo",
        "x": -2.50,
        "y": 10.50,
        "yaw": 1.5708,
    },
    "board_room": {
        "name": "Board Room",
        "x": -23.00,
        "y": 9.50,
        "yaw": 3.1415,
    },
    "accounts_ws": {
        "name": "Accounts Workstation",
        "x": -47.00,
        "y": 1.50,
        "yaw": 3.1415,
    }
}

# Corridor intermediate waypoints
CORRIDOR_STEP_SOUTH_OF_LIVING_DEMO = (-2.50, 7.40, -1.5708)
CORRIDOR_CENTRE_BELOW_BOARD_ROOM   = (-23.00, 7.40, 1.5708)
CORRIDOR_WEST_PASSAGE_ENTRY        = (-33.50, 7.25, -1.5708)
PROJECT_PASSAGEWAY_WEST            = (-33.50, 1.80, 3.1415)
CORRIDOR_CENTRE_ABOVE_LOBBY        = (-2.00,  7.25, -1.5708)

LEGS = [
    {
        "leg_id": 1,
        "name": "Leg 1: Lobby -> Living Demo",
        "dest_key": "living_demo",
        "timeout": 90.0,
        "waypoints": []
    },
    {
        "leg_id": 2,
        "name": "Leg 2: Living Demo -> Board Room",
        "dest_key": "board_room",
        "timeout": 120.0,
        "waypoints": [
            CORRIDOR_STEP_SOUTH_OF_LIVING_DEMO,
            CORRIDOR_CENTRE_BELOW_BOARD_ROOM,
        ]
    },
    {
        "leg_id": 3,
        "name": "Leg 3: Board Room -> Accounts",
        "dest_key": "accounts_ws",
        "timeout": 150.0,
        "waypoints": [
            CORRIDOR_CENTRE_BELOW_BOARD_ROOM,
            CORRIDOR_WEST_PASSAGE_ENTRY,
            PROJECT_PASSAGEWAY_WEST,
        ]
    },
    {
        "leg_id": 4,
        "name": "Leg 4: Accounts -> Lobby",
        "dest_key": "lobby",
        "timeout": 250.0,
        "waypoints": [
            PROJECT_PASSAGEWAY_WEST,
            CORRIDOR_WEST_PASSAGE_ENTRY,
            CORRIDOR_CENTRE_ABOVE_LOBBY,
        ]
    }
]

class MetricsMonitor(Node):
    def __init__(self):
        super().__init__('phase2_metrics_monitor')
        self.min_lidar_range = float('inf')
        self.max_cmd_lin = 0.0
        self.max_cmd_ang = 0.0
        self.last_cmd_time = 0.0
        
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

    def scan_cb(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r)]
        if valid:
            m = min(valid)
            if m < self.min_lidar_range:
                self.min_lidar_range = m

    def cmd_cb(self, msg):
        lin = math.hypot(msg.linear.x, msg.linear.y)
        ang = abs(msg.angular.z)
        if lin > self.max_cmd_lin:
            self.max_cmd_lin = lin
        if ang > self.max_cmd_ang:
            self.max_cmd_ang = ang
        self.last_cmd_time = time.time()

    def reset_metrics(self):
        self.min_lidar_range = float('inf')
        self.max_cmd_lin = 0.0
        self.max_cmd_ang = 0.0

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
    except Exception:
        return None, None, None

def get_gz_world_pose():
    # Direct gz topic query
    try:
        res = subprocess.run(
            ["gz", "topic", "-e", "-t", "/world/robot_world/pose/info", "-n", "1"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3
        )
        lines = res.stdout.splitlines()
        in_robot = False
        x, y, z, qz, qw = None, None, None, None, None
        for i, line in enumerate(lines):
            if 'name: "four_wheel_robot"' in line:
                in_robot = True
            elif in_robot and 'name:' in line and 'four_wheel_robot' not in line:
                break
            elif in_robot:
                m_x = re.search(r'x:\s*([-\d\.e]+)', line)
                m_y = re.search(r'y:\s*([-\d\.e]+)', line)
                m_z = re.search(r'z:\s*([-\d\.e]+)', line)
                m_w = re.search(r'w:\s*([-\d\.e]+)', line)
                if 'position' in lines[max(0, i-1)] or 'position' in lines[max(0, i-2)]:
                    if m_x and x is None: x = float(m_x.group(1))
                    if m_y and y is None: y = float(m_y.group(1))
                    if m_z and z is None: z = float(m_z.group(1))
                if 'orientation' in lines[max(0, i-1)] or 'orientation' in lines[max(0, i-2)] or 'orientation' in lines[max(0, i-3)] or 'orientation' in lines[max(0, i-4)]:
                    if m_z and qz is None: qz = float(m_z.group(1))
                    if m_w and qw is None: qw = float(m_w.group(1))

        yaw = None
        if qz is not None and qw is not None:
            yaw = 2.0 * math.atan2(qz, qw)
        return x, y, yaw
    except Exception as e:
        print(f"gz query error: {e}")
        return None, None, None

def run_sequence(seq_id, output_path):
    print(f"\n=======================================================")
    print(f" STARTING SEQUENCE {seq_id} - FULL 4-LEG TOUR")
    print(f"=======================================================")

    rclpy.init()
    navigator = BasicNavigator()
    monitor = MetricsMonitor()

    print("Waiting for Nav2 stack...")
    navigator.waitUntilNav2Active(localizer="robot_localization")
    print("Nav2 active.")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, navigator)

    # Let buffers initialize
    for _ in range(15):
        rclpy.spin_once(monitor, timeout_sec=0.1)
        rclpy.spin_once(navigator, timeout_sec=0.1)

    seq_results = []
    overall_success = True

    for leg in LEGS:
        print(f"\n--- Starting {leg['name']} ---")
        dest = DESTINATIONS[leg["dest_key"]]
        gx, gy, gyaw = dest["x"], dest["y"], dest["yaw"]
        print(f"Target: ({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f})")

        monitor.reset_metrics()

        start_tf_x, start_tf_y, start_tf_yaw = get_current_tf_pose(tf_buffer)
        start_gz_x, start_gz_y, start_gz_yaw = get_gz_world_pose()
        print(f"Start Nav2 Pose: ({start_tf_x:.3f}, {start_tf_y:.3f})" if start_tf_x else "Start Nav2: None")
        print(f"Start Gazebo Pose: ({start_gz_x:.3f}, {start_gz_y:.3f})" if start_gz_x else "Start Gazebo: None")

        if leg["waypoints"]:
            poses = []
            for (wx, wy, wyaw) in leg["waypoints"]:
                poses.append(make_pose_stamped(navigator, wx, wy, wyaw))
            poses.append(make_pose_stamped(navigator, gx, gy, gyaw))
            print(f"Sending NavigateThroughPoses with {len(poses)} waypoints...")
            navigator.goThroughPoses(poses)
        else:
            goal_pose = make_pose_stamped(navigator, gx, gy, gyaw)
            print(f"Sending goToPose...")
            navigator.goToPose(goal_pose)

        start_time = time.time()
        max_recoveries = 0
        leg_trajectory = []
        timeout = leg["timeout"]

        while not navigator.isTaskComplete():
            rclpy.spin_once(monitor, timeout_sec=0.1)
            rclpy.spin_once(navigator, timeout_sec=0.1)

            elapsed = time.time() - start_time
            feedback = navigator.getFeedback()
            tf_x, tf_y, tf_yaw = get_current_tf_pose(tf_buffer)

            if feedback:
                cx = feedback.current_pose.pose.position.x
                cy = feedback.current_pose.pose.position.y
                dist_rem = feedback.distance_remaining
                recovers = feedback.number_of_recoveries
                if recovers > max_recoveries:
                    max_recoveries = recovers

                leg_trajectory.append({
                    "time": round(elapsed, 2),
                    "nav2_x": round(cx, 3),
                    "nav2_y": round(cy, 3),
                    "tf_x": round(tf_x, 3) if tf_x else None,
                    "tf_y": round(tf_y, 3) if tf_y else None,
                    "dist_rem": round(dist_rem, 3),
                    "recovers": recovers
                })

                if len(leg_trajectory) % 10 == 0 or dist_rem < 0.6:
                    print(f"  [{elapsed:5.1f}s] Pose: ({cx:6.2f}, {cy:6.2f}) | Rem: {dist_rem:5.2f}m | Rec: {recovers} | MinScan: {monitor.min_lidar_range:4.2f}m")

            if elapsed > timeout:
                print(f"  [TIMEOUT] Exceeded {timeout}s! Cancelling...")
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
            status_str = "TIMEOUT" if elapsed >= timeout else "UNKNOWN"

        for _ in range(10):
            rclpy.spin_once(monitor, timeout_sec=0.05)
            rclpy.spin_once(navigator, timeout_sec=0.05)

        final_tf_x, final_tf_y, final_tf_yaw = get_current_tf_pose(tf_buffer)
        final_gz_x, final_gz_y, final_gz_yaw = get_gz_world_pose()

        # Compute position disagreement between Nav2 and Gazebo
        pose_disagreement = None
        if final_tf_x is not None and final_gz_x is not None:
            pose_disagreement = math.hypot(final_tf_x - final_gz_x, final_tf_y - final_gz_y)

        # Distance from requested goal to physical Gazebo pose
        physical_goal_dist = None
        if final_gz_x is not None:
            physical_goal_dist = math.hypot(final_gz_x - gx, final_gz_y - gy)

        # Check for persistent wall wedging or contact
        # If min_lidar_range < 0.20m or physical_goal_dist > 0.35m when SUCCEEDED
        wall_contact = False
        if monitor.min_lidar_range < 0.18:
            wall_contact = True

        leg_data = {
            "sequence": seq_id,
            "leg_id": leg["leg_id"],
            "name": leg["name"],
            "status": status_str,
            "elapsed_sec": round(elapsed, 2),
            "requested_goal": {"x": gx, "y": gy, "yaw": gyaw},
            "final_nav2_pose": {
                "x": round(final_tf_x, 4) if final_tf_x else None,
                "y": round(final_tf_y, 4) if final_tf_y else None,
                "yaw": round(final_tf_yaw, 4) if final_tf_yaw else None
            },
            "final_gazebo_pose": {
                "x": round(final_gz_x, 4) if final_gz_x else None,
                "y": round(final_gz_y, 4) if final_gz_y else None,
                "yaw": round(final_gz_yaw, 4) if final_gz_yaw else None
            },
            "pose_disagreement_m": round(pose_disagreement, 4) if pose_disagreement is not None else None,
            "physical_goal_dist_m": round(physical_goal_dist, 4) if physical_goal_dist is not None else None,
            "min_lidar_range_m": round(monitor.min_lidar_range, 3) if monitor.min_lidar_range != float('inf') else None,
            "max_cmd_lin_ms": round(monitor.max_cmd_lin, 3),
            "max_cmd_ang_rads": round(monitor.max_cmd_ang, 3),
            "recovery_count": max_recoveries,
            "wall_contact_detected": wall_contact,
            "trajectory_points": len(leg_trajectory)
        }
        seq_results.append(leg_data)

        print("-" * 60)
        print(f" Leg {leg['leg_id']} Finished:")
        print(f"   Status:                   {status_str}")
        print(f"   Duration:                 {elapsed:.1f}s")
        print(f"   Nav2 Pose:                ({final_tf_x:.3f}, {final_tf_y:.3f})" if final_tf_x else "   Nav2 Pose: N/A")
        print(f"   Gazebo Physical Pose:     ({final_gz_x:.3f}, {final_gz_y:.3f})" if final_gz_x else "   Gazebo Pose: N/A")
        print(f"   Goal Dist (Physical):     {physical_goal_dist:.3f}m" if physical_goal_dist is not None else "   Goal Dist: N/A")
        print(f"   Nav2-Gazebo Disagreement: {pose_disagreement:.4f}m" if pose_disagreement is not None else "   Disagreement: N/A")
        print(f"   Min LiDAR Range:          {monitor.min_lidar_range:.3f}m")
        print(f"   Max Cmd Lin:              {monitor.max_cmd_lin:.2f} m/s")
        print(f"   Max Cmd Ang:              {monitor.max_cmd_ang:.2f} rad/s")
        print(f"   Recoveries:               {max_recoveries}")
        print(f"   Wall Contact:             {'YES' if wall_contact else 'NO'}")
        print("-" * 60)

        if status_str != "SUCCEEDED":
            print(f"[ABORT] Leg {leg['leg_id']} failed or timed out! Stopping sequence.")
            overall_success = False
            break

        time.sleep(2.0)

    monitor.destroy_node()
    rclpy.shutdown()

    with open(output_path, "w") as f:
        json.dump(seq_results, f, indent=2)
    print(f"Sequence {seq_id} results written to {output_path}")

    return overall_success, seq_results

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seq", type=int, default=1, help="Sequence number (1, 2, or 3)")
    parser.add_argument("--output", type=str, default="/tmp/phase2_seq_result.json", help="Output JSON path")
    args = parser.parse_args()

    succ, _ = run_sequence(args.seq, args.output)
    sys.exit(0 if succ else 1)

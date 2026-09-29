#!/usr/bin/env python3
"""
Step 3 Independent Verification Test Harness
Executes:
  Part 1: Living Demo -> Board Room (3 runs from clean, consistent start)
  Part 2: Accounts -> Lobby (3 runs from clean, consistent start)
  Part 3: Complete 4-leg tour (1 full tour from clean start)

Records for every run:
  - Goal result & duration
  - Final physical Gazebo pose & Nav2 pose
  - Position disagreement (m)
  - Distance from requested goal to physical pose (m)
  - Minimum LiDAR range encountered (m)
  - Recovery count
  - Physical wall contacts
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
from nav2_msgs.srv import ClearEntireCostmap
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

# Corridor waypoints (updated with smallest justified fix)
CORRIDOR_STEP_SOUTH_OF_LIVING_DEMO = (-2.50, 7.40, -1.5708)
CORRIDOR_CENTRE_BELOW_BOARD_ROOM   = (-23.00, 7.40, 1.5708)
CORRIDOR_WEST_PASSAGE_ENTRY        = (-33.50, 7.25, -1.5708)
PROJECT_PASSAGEWAY_WEST            = (-33.50, 1.80, 3.1415)
CORRIDOR_CENTRE_ABOVE_LOBBY        = (-2.00,  7.25, -1.5708)
CORRIDOR_WEST_APPROACH_ACCOUNTS    = (-44.00, 1.50, 3.1415)

ROUTES = {
    "living_to_board": {
        "name": "Living Demo -> Board Room",
        "start_pose": {"x": -2.50, "y": 10.50, "yaw": 1.5708},
        "dest_key": "board_room",
        "timeout": 120.0,
        "waypoints": [
            CORRIDOR_STEP_SOUTH_OF_LIVING_DEMO,
            CORRIDOR_CENTRE_BELOW_BOARD_ROOM,
        ]
    },
    "accounts_to_lobby": {
        "name": "Accounts -> Lobby",
        "start_pose": {"x": -47.00, "y": 1.50, "yaw": 3.1415},
        "dest_key": "lobby",
        "timeout": 200.0,
        "waypoints": [
            PROJECT_PASSAGEWAY_WEST,
            CORRIDOR_WEST_PASSAGE_ENTRY,
            CORRIDOR_CENTRE_ABOVE_LOBBY,
        ]
    }
}

TOUR_LEGS = [
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
            CORRIDOR_WEST_APPROACH_ACCOUNTS,
        ]
    },
    {
        "leg_id": 4,
        "name": "Leg 4: Accounts -> Lobby",
        "dest_key": "lobby",
        "timeout": 200.0,
        "waypoints": [
            PROJECT_PASSAGEWAY_WEST,
            CORRIDOR_WEST_PASSAGE_ENTRY,
            CORRIDOR_CENTRE_ABOVE_LOBBY,
        ]
    }
]

class MetricsMonitor(Node):
    def __init__(self):
        super().__init__('step3_metrics_monitor')
        self.min_lidar_range = float('inf')
        self.max_cmd_lin = 0.0
        self.max_cmd_ang = 0.0
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

    def reset_metrics(self):
        self.min_lidar_range = float('inf')
        self.max_cmd_lin = 0.0
        self.max_cmd_ang = 0.0

def set_gazebo_pose(x, y, yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    req = f'name: "four_wheel_robot", position: {{x: {x}, y: {y}, z: 0.1}}, orientation: {{z: {qz}, w: {qw}}}'
    cmd = ["gz", "service", "-s", "/world/robot_world/set_pose",
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "2000", "--req", req]
    subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)

def get_gz_world_pose():
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
    except Exception:
        return None, None, None

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

def clear_costmaps(node):
    for srv_name in ['/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap']:
        client = node.create_client(ClearEntireCostmap, srv_name)
        if client.wait_for_service(timeout_sec=2.0):
            req = ClearEntireCostmap.Request()
            client.call_async(req)
    time.sleep(1.0)

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

def execute_leg(navigator, monitor, tf_buffer, leg_config):
    dest = DESTINATIONS[leg_config["dest_key"]]
    gx, gy, gyaw = dest["x"], dest["y"], dest["yaw"]
    timeout = leg_config["timeout"]

    print(f"\n>>> Running: {leg_config['name']}")
    print(f"    Target: ({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f})")

    monitor.reset_metrics()

    poses = []
    if leg_config["waypoints"]:
        for (wx, wy, wyaw) in leg_config["waypoints"]:
            poses.append(make_pose_stamped(navigator, wx, wy, wyaw))
        poses.append(make_pose_stamped(navigator, gx, gy, gyaw))
        navigator.goThroughPoses(poses)
    else:
        goal_pose = make_pose_stamped(navigator, gx, gy, gyaw)
        navigator.goToPose(goal_pose)

    start_time = time.time()
    max_recoveries = 0
    trajectory = []

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

            trajectory.append({
                "time": round(elapsed, 2),
                "x": round(cx, 3),
                "y": round(cy, 3),
                "dist_rem": round(dist_rem, 3),
                "recovers": recovers
            })

            if len(trajectory) % 10 == 0 or dist_rem < 0.6:
                print(f"    [{elapsed:5.1f}s] Pose: ({cx:6.2f}, {cy:6.2f}) | Rem: {dist_rem:5.2f}m | Rec: {recovers} | MinScan: {monitor.min_lidar_range:4.2f}m")

        if elapsed > timeout:
            print(f"    [TIMEOUT] Exceeded {timeout}s! Cancelling...")
            navigator.cancelTask()
            break

    elapsed = time.time() - start_time
    result = navigator.getResult()

    if elapsed >= timeout:
        status_str = "TIMEOUT"
    elif result == TaskResult.SUCCEEDED:
        status_str = "SUCCEEDED"
    elif result == TaskResult.CANCELED:
        status_str = "CANCELED"
    else:
        status_str = "FAILED"

    for _ in range(10):
        rclpy.spin_once(monitor, timeout_sec=0.05)
        rclpy.spin_once(navigator, timeout_sec=0.05)

    final_tf_x, final_tf_y, final_tf_yaw = get_current_tf_pose(tf_buffer)
    final_gz_x, final_gz_y, final_gz_yaw = get_gz_world_pose()

    pose_disagreement = None
    if final_tf_x is not None and final_gz_x is not None:
        pose_disagreement = math.hypot(final_tf_x - final_gz_x, final_tf_y - final_gz_y)

    physical_goal_dist = None
    if final_gz_x is not None:
        physical_goal_dist = math.hypot(final_gz_x - gx, final_gz_y - gy)

    wall_contact = (monitor.min_lidar_range < 0.18)

    leg_result = {
        "name": leg_config["name"],
        "status": status_str,
        "elapsed_sec": round(elapsed, 2),
        "target": {"x": gx, "y": gy, "yaw": gyaw},
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
        "wall_contact": wall_contact
    }

    print(f"    RESULT: {status_str} in {elapsed:.1f}s | GoalDist: {physical_goal_dist:.3f}m | Disagreement: {pose_disagreement:.4f}m | MinScan: {monitor.min_lidar_range:.2f}m")
    return leg_result

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", type=str, required=True, choices=["living_to_board", "accounts_to_lobby", "full_tour"],
                        help="Test mode to run")
    parser.add_argument("--runs", type=int, default=3, help="Number of repetitions")
    parser.add_argument("--output", type=str, default="/tmp/step3_result.json", help="Output JSON path")
    args = parser.parse_args()

    rclpy.init()
    navigator = BasicNavigator()
    monitor = MetricsMonitor()

    print("Waiting for Nav2 stack...")
    navigator.waitUntilNav2Active(localizer="robot_localization")
    print("Nav2 active.")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, navigator)

    for _ in range(15):
        rclpy.spin_once(monitor, timeout_sec=0.1)
        rclpy.spin_once(navigator, timeout_sec=0.1)

    all_runs = []

    if args.test in ["living_to_board", "accounts_to_lobby"]:
        cfg = ROUTES[args.test]
        for run_idx in range(1, args.runs + 1):
            print(f"\n=======================================================")
            print(f" {cfg['name']} - RUN {run_idx}/{args.runs}")
            print(f"=======================================================")

            # Set clean initial pose for the route
            sp = cfg["start_pose"]
            set_gazebo_pose(sp["x"], sp["y"], sp["yaw"])
            clear_costmaps(monitor)

            # Let sensors and TF settle
            for _ in range(20):
                rclpy.spin_once(monitor, timeout_sec=0.1)
                rclpy.spin_once(navigator, timeout_sec=0.1)

            res = execute_leg(navigator, monitor, tf_buffer, cfg)
            res["run"] = run_idx
            all_runs.append(res)

            time.sleep(2.0)

    elif args.test == "full_tour":
        print(f"\n=======================================================")
        print(f" FULL 4-LEG TOUR")
        print(f"=======================================================")

        # Clean start at Lobby
        set_gazebo_pose(-1.80, 0.00, 1.5708)
        clear_costmaps(monitor)
        for _ in range(20):
            rclpy.spin_once(monitor, timeout_sec=0.1)
            rclpy.spin_once(navigator, timeout_sec=0.1)

        for leg_cfg in TOUR_LEGS:
            res = execute_leg(navigator, monitor, tf_buffer, leg_cfg)
            all_runs.append(res)
            if res["status"] != "SUCCEEDED":
                print(f"[ABORT] Leg {leg_cfg['leg_id']} failed! Stopping tour.")
                break
            time.sleep(2.0)

    monitor.destroy_node()
    rclpy.shutdown()

    with open(args.output, "w") as f:
        json.dump(all_runs, f, indent=2)
    print(f"\nResults written to {args.output}")

    all_passed = all(r["status"] == "SUCCEEDED" and (r["physical_goal_dist_m"] or 0) <= 0.35 and (r["pose_disagreement_m"] or 0) <= 0.10 and not r["wall_contact"] for r in all_runs)
    sys.exit(0 if all_passed else 1)

if __name__ == "__main__":
    main()

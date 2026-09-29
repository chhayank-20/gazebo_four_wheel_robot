#!/usr/bin/env python3
"""
Phase 5 Diagnostic Harness: Accounts Goal Accuracy & Clearance Audit
Tests Board Room -> Accounts route across 3 repetitions with synchronized pose sampling
and obstacle coordinate projection.
"""

import sys
import os
import time
import math
import json
import re
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.srv import ClearEntireCostmap

def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

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

def set_gazebo_pose(x, y, yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    req = f'name: "four_wheel_robot", position: {{x: {x}, y: {y}, z: 0.1}}, orientation: {{z: {qz}, w: {qw}}}'
    cmd = ["gz", "service", "-s", "/world/robot_world/set_pose",
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "2000", "--req", req]
    subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)

class DiagnosticMonitor(Node):
    def __init__(self):
        super().__init__('phase5_diagnostic_monitor')
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        self.latest_odom = None
        self.latest_scan = None
        self.latest_cmd = None
        
        self.min_range = float('inf')
        self.min_range_info = None

    def odom_cb(self, msg):
        self.latest_odom = msg

    def scan_cb(self, msg):
        self.latest_scan = msg
        for i, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max:
                if r < self.min_range:
                    self.min_range = r
                    angle = msg.angle_min + i * msg.angle_increment
                    self.min_range_info = {
                        "range": r,
                        "angle_rad": angle,
                        "angle_deg": math.degrees(angle),
                        "beam_index": i
                    }

    def cmd_cb(self, msg):
        self.latest_cmd = msg

    def reset_metrics(self):
        self.min_range = float('inf')
        self.min_range_info = None

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

def get_tf_pose(tf_buffer):
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

def run_test(num_runs=3):
    rclpy.init()
    monitor = DiagnosticMonitor()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, monitor)
    navigator = BasicNavigator()

    # Route definitions
    # Board Room -> Accounts
    start_pose = {"x": -23.00, "y": 9.50, "yaw": 3.1415}
    goal_pose = {"x": -47.00, "y": 1.50, "yaw": 3.1415}
    waypoints = [
        (-23.00, 7.40, 1.5708),
        (-33.50, 7.25, -1.5708),
        (-33.50, 1.80, 3.1415),
    ]

    print(f"\n======================================================================")
    print(f" PHASE 5: ACCOUNTS GOAL ACCURACY & CLEARANCE AUDIT ({num_runs} RUNS)")
    print(f" Target: ({goal_pose['x']}, {goal_pose['y']}, yaw={goal_pose['yaw']:.4f})")
    print(f"======================================================================")

    results = []

    for run_idx in range(1, num_runs + 1):
        print(f"\n--- [RUN {run_idx}/{num_runs}] Preparing Route: Board Room -> Accounts ---")
        
        # Reset robot to Board Room
        set_gazebo_pose(start_pose["x"], start_pose["y"], start_pose["yaw"])
        time.sleep(1.0)
        
        # Clear costmaps
        clear_costmaps(monitor)
        monitor.reset_metrics()

        # Spin to sync
        for _ in range(20):
            rclpy.spin_once(monitor, timeout_sec=0.05)

        cur_x, cur_y, cur_yaw = get_tf_pose(tf_buffer)
        print(f"  Robot reset to: ({cur_x:.2f}, {cur_y:.2f}, yaw={cur_yaw:.2f} rad)")

        # Build poses
        pose_list = []
        for (wx, wy, wyaw) in waypoints:
            pose_list.append(make_pose_stamped(navigator, wx, wy, wyaw))
        pose_list.append(make_pose_stamped(navigator, goal_pose["x"], goal_pose["y"], goal_pose["yaw"]))

        print(f"  Sending NavigateThroughPoses with {len(pose_list)} poses...")
        navigator.goThroughPoses(pose_list)

        start_time = time.time()
        timeout = 150.0
        
        instant_tf = None
        instant_odom = None
        instant_gz = None
        instant_dist = None
        instant_yaw_err = None

        min_scan_obstacle_map = None

        while not navigator.isTaskComplete():
            rclpy.spin_once(monitor, timeout_sec=0.05)
            rclpy.spin_once(navigator, timeout_sec=0.05)
            
            elapsed = time.time() - start_time
            cx, cy, cyaw = get_tf_pose(tf_buffer)

            if cx is not None:
                dist_rem = math.hypot(goal_pose["x"] - cx, goal_pose["y"] - cy)
            else:
                dist_rem = 999.0

            # Compute obstacle coordinate for min range
            if monitor.min_range_info and cx is not None:
                r = monitor.min_range_info["range"]
                ang = monitor.min_range_info["angle_rad"]
                x_b = 0.25 + r * math.cos(ang)
                y_b = r * math.sin(ang)
                x_m = cx + x_b * math.cos(cyaw) - y_b * math.sin(cyaw)
                y_m = cy + x_b * math.sin(cyaw) + y_b * math.cos(cyaw)
                min_scan_obstacle_map = {
                    "range": round(r, 3),
                    "beam_deg": round(monitor.min_range_info["angle_deg"], 1),
                    "robot_pose": (round(cx, 2), round(cy, 2), round(cyaw, 2)),
                    "obs_map_coord": (round(x_m, 2), round(y_m, 2))
                }

            if int(elapsed * 10) % 20 == 0:
                print(f"    [{elapsed:5.1f}s] Pose: ({cx:6.2f}, {cy:6.2f}) | Rem: {dist_rem:5.2f}m | MinScan: {monitor.min_range:4.2f}m")

            if elapsed > timeout:
                print(f"    [TIMEOUT] Exceeded {timeout}s!")
                navigator.cancelTask()
                break

        elapsed = time.time() - start_time
        result = navigator.getResult()
        status_str = "SUCCEEDED" if result == TaskResult.SUCCEEDED else "FAILED"

        # Capture pose AT THE EXACT INSTANT Nav2 finishes
        inst_tx, inst_ty, inst_tyaw = get_tf_pose(tf_buffer)
        if monitor.latest_odom:
            inst_ox = monitor.latest_odom.pose.pose.position.x
            inst_oy = monitor.latest_odom.pose.pose.position.y
            oq_z = monitor.latest_odom.pose.pose.orientation.z
            oq_w = monitor.latest_odom.pose.pose.orientation.w
            inst_oyaw = 2.0 * math.atan2(oq_z, oq_w)
        else:
            inst_ox, inst_oy, inst_oyaw = None, None, None

        inst_dist = math.hypot(inst_tx - goal_pose["x"], inst_ty - goal_pose["y"])
        inst_yaw_err = abs(wrap_angle(inst_tyaw - goal_pose["yaw"]))

        # Wait 1.0s for robot to settle fully to rest (v=0, w=0)
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(monitor, timeout_sec=0.05)

        settled_tx, settled_ty, settled_tyaw = get_tf_pose(tf_buffer)
        settled_gx, settled_gy, settled_gyaw = get_gz_world_pose()

        settled_dist = math.hypot(settled_tx - goal_pose["x"], settled_ty - goal_pose["y"])
        settled_yaw_err_deg = math.degrees(abs(wrap_angle(settled_tyaw - goal_pose["yaw"])))

        disagreement = None
        if settled_tx is not None and settled_gx is not None:
            disagreement = math.hypot(settled_tx - settled_gx, settled_ty - settled_gy)

        gz_goal_dist = None
        if settled_gx is not None:
            gz_goal_dist = math.hypot(settled_gx - goal_pose["x"], settled_gy - goal_pose["y"])

        run_res = {
            "run": run_idx,
            "status": status_str,
            "elapsed_sec": round(elapsed, 2),
            "instant_finish_pose": {
                "tf_x": round(inst_tx, 4),
                "tf_y": round(inst_ty, 4),
                "tf_yaw": round(inst_tyaw, 4),
                "goal_dist_m": round(inst_dist, 4),
                "yaw_error_deg": round(math.degrees(inst_yaw_err), 2)
            },
            "settled_pose": {
                "tf_x": round(settled_tx, 4),
                "tf_y": round(settled_ty, 4),
                "tf_yaw": round(settled_tyaw, 4),
                "goal_dist_m": round(settled_dist, 4),
                "yaw_error_deg": round(settled_yaw_err_deg, 2)
            },
            "gazebo_world_pose": {
                "x": round(settled_gx, 4) if settled_gx else None,
                "y": round(settled_gy, 4) if settled_gy else None,
                "yaw": round(settled_gyaw, 4) if settled_gyaw else None,
                "goal_dist_m": round(gz_goal_dist, 4) if gz_goal_dist else None
            },
            "disagreement_m": round(disagreement, 5) if disagreement is not None else None,
            "min_lidar_range_m": round(monitor.min_range, 3),
            "min_scan_obstacle": min_scan_obstacle_map
        }
        results.append(run_res)

        print(f"\n  === RUN {run_idx} SUMMARY ===")
        print(f"  Status: {status_str} in {elapsed:.1f}s")
        print(f"  Instant Finish TF: ({inst_tx:.4f}, {inst_ty:.4f}, yaw={inst_tyaw:.4f}) | GoalDist: {inst_dist:.4f}m | YawErr: {math.degrees(inst_yaw_err):.2f}°")
        print(f"  Settled TF:        ({settled_tx:.4f}, {settled_ty:.4f}, yaw={settled_tyaw:.4f}) | GoalDist: {settled_dist:.4f}m | YawErr: {settled_yaw_err_deg:.2f}°")
        print(f"  Gazebo GroundTruth:({settled_gx:.4f}, {settled_gy:.4f}, yaw={settled_gyaw:.4f}) | GoalDist: {gz_goal_dist:.4f}m | Disagreement: {disagreement:.5f}m")
        print(f"  Min LiDAR Range:   {monitor.min_range:.3f}m | Closest Obs Map Coord: {min_scan_obstacle_map['obs_map_coord']}")

    with open('/workspace/gazebo_four_wheel_robot/phase5_accounts_diag.json', 'w') as f:
        json.dump(results, f, indent=2)
    print("\nSaved diagnostics to phase5_accounts_diag.json")

    rclpy.shutdown()

if __name__ == '__main__':
    run_test(3)

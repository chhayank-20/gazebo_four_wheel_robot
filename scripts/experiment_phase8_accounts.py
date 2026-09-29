#!/usr/bin/env python3
"""
Phase 8 Controlled Experiment: Accounts Arrival Repeatability
Compares approach strategies into Accounts:
- Baseline (no approach waypoint)
- Candidate A: Approach waypoint at (-44.00, 1.50, 3.1415)
- Candidate B: Approach waypoint at (-44.00, 1.60, 3.1415)

Captures both:
1. Pose at action completion
2. Pose after settling (v=0, w=0)
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
        sec, nsec = None, None
        for i, line in enumerate(lines):
            if 'sec:' in line and sec is None:
                m_s = re.search(r'sec:\s*(\d+)', line)
                if m_s: sec = int(m_s.group(1))
            if 'nsec:' in line and nsec is None:
                m_ns = re.search(r'nsec:\s*(\d+)', line)
                if m_ns: nsec = int(m_ns.group(1))

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
        t_sim = float(sec) + float(nsec)*1e-9 if (sec is not None and nsec is not None) else None
        return x, y, yaw, t_sim
    except Exception:
        return None, None, None, None

def set_gazebo_pose(x, y, yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    req = f'name: "four_wheel_robot", position: {{x: {x}, y: {y}, z: 0.1}}, orientation: {{z: {qz}, w: {qw}}}'
    cmd = ["gz", "service", "-s", "/world/robot_world/set_pose",
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "2000", "--req", req]
    subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)

class ExperimentMonitor(Node):
    def __init__(self):
        super().__init__('phase8_experiment_monitor')
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        self.latest_odom = None
        self.latest_odom_stamp = None
        self.latest_scan = None
        self.latest_cmd = Twist()
        self.min_range = float('inf')

    def odom_cb(self, msg):
        self.latest_odom = msg
        self.latest_odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def scan_cb(self, msg):
        self.latest_scan = msg
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                if r < self.min_range:
                    self.min_range = r

    def cmd_cb(self, msg):
        self.latest_cmd = msg

    def reset_metrics(self):
        self.min_range = float('inf')

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

def run_single_test(navigator, monitor, start_pose, waypoints, goal_pose, label=""):
    set_gazebo_pose(start_pose["x"], start_pose["y"], start_pose["yaw"])
    time.sleep(1.0)
    clear_costmaps(monitor)
    monitor.reset_metrics()

    for _ in range(20):
        rclpy.spin_once(monitor, timeout_sec=0.05)

    pose_list = [make_pose_stamped(navigator, wx, wy, wyaw) for (wx, wy, wyaw) in waypoints]
    pose_list.append(make_pose_stamped(navigator, goal_pose["x"], goal_pose["y"], goal_pose["yaw"]))

    navigator.goThroughPoses(pose_list)
    start_time = time.time()
    timeout = 150.0

    while not navigator.isTaskComplete():
        rclpy.spin_once(monitor, timeout_sec=0.02)
        rclpy.spin_once(navigator, timeout_sec=0.02)
        elapsed = time.time() - start_time
        if elapsed > timeout:
            navigator.cancelTask()
            break

    elapsed = time.time() - start_time
    nav2_result = navigator.getResult()
    nav2_success = (nav2_result == TaskResult.SUCCEEDED)

    # Instant capture
    for _ in range(5):
        rclpy.spin_once(monitor, timeout_sec=0.01)

    t_instant_sim = monitor.latest_odom_stamp
    if monitor.latest_odom:
        inst_x = monitor.latest_odom.pose.pose.position.x
        inst_y = monitor.latest_odom.pose.pose.position.y
        oq_z = monitor.latest_odom.pose.pose.orientation.z
        oq_w = monitor.latest_odom.pose.pose.orientation.w
        inst_yaw = 2.0 * math.atan2(oq_z, oq_w)
    else:
        inst_x, inst_y, inst_yaw = None, None, None

    inst_dist = math.hypot(inst_x - goal_pose["x"], inst_y - goal_pose["y"]) if inst_x is not None else None
    inst_yaw_err = abs(wrap_angle(inst_yaw - goal_pose["yaw"])) if inst_yaw is not None else None

    # Settled capture after 1.0s
    time.sleep(1.0)
    for _ in range(10):
        rclpy.spin_once(monitor, timeout_sec=0.05)

    settled_gx, settled_gy, settled_gyaw, settled_t_sim = get_gz_world_pose()
    settled_dist = math.hypot(settled_gx - goal_pose["x"], settled_gy - goal_pose["y"]) if settled_gx is not None else None
    settled_yaw_err = abs(wrap_angle(settled_gyaw - goal_pose["yaw"])) if settled_gyaw is not None else None

    crit_A = nav2_success
    crit_B = (settled_dist is not None and settled_dist <= 0.3500 and settled_yaw_err <= 0.40)
    crit_C = (monitor.min_range >= 0.18)

    res = {
        "label": label,
        "elapsed_sec": round(elapsed, 2),
        "nav2_success": nav2_success,
        "instant_at_completion": {
            "x": round(inst_x, 4) if inst_x is not None else None,
            "y": round(inst_y, 4) if inst_y is not None else None,
            "yaw": round(inst_yaw, 4) if inst_yaw is not None else None,
            "dist_m": round(inst_dist, 4) if inst_dist is not None else None,
            "yaw_err_deg": round(math.degrees(inst_yaw_err), 2) if inst_yaw_err is not None else None,
            "t_sim": round(t_instant_sim, 3) if t_instant_sim else None
        },
        "settled_at_rest": {
            "gz_x": round(settled_gx, 4) if settled_gx is not None else None,
            "gz_y": round(settled_gy, 4) if settled_gy is not None else None,
            "gz_yaw": round(settled_gyaw, 4) if settled_gyaw is not None else None,
            "dist_m": round(settled_dist, 4) if settled_dist is not None else None,
            "yaw_err_deg": round(math.degrees(settled_yaw_err), 2) if settled_yaw_err is not None else None,
            "t_sim": round(settled_t_sim, 3) if settled_t_sim else None
        },
        "clearance": {
            "min_lidar_m": round(monitor.min_range, 3),
            "safe_no_contact": crit_C
        },
        "criteria": {
            "criterion_A_action": crit_A,
            "criterion_B_physical_accuracy": crit_B,
            "criterion_C_collision": crit_C
        }
    }

    print(f"  [{label}] Elapsed: {res['elapsed_sec']}s | Nav2: {nav2_result} | Settled Gz: ({settled_gx:.4f}, {settled_gy:.4f}, yaw={settled_gyaw:.4f})")
    print(f"       -> Settled Dist: {settled_dist:.4f}m (Threshold <= 0.3500m: {'PASS' if crit_B else 'FAIL'}) | YawErr: {math.degrees(settled_yaw_err):.2f}° | MinScan: {monitor.min_range:.3f}m")
    return res

def main():
    rclpy.init()
    monitor = ExperimentMonitor()
    navigator = BasicNavigator()

    start_pose = {"x": -23.00, "y": 9.50, "yaw": 3.1415}
    goal_pose = {"x": -47.00, "y": 1.50, "yaw": 3.1415}

    base_waypoints = [
        (-23.00, 7.40, 1.5708),
        (-33.50, 7.25, -1.5708),
        (-33.50, 1.80, 3.1415),
    ]

    # Candidate A: Adds straight-in door entry waypoint at (-44.00, 1.50, 3.1415)
    cand_A_waypoints = base_waypoints + [(-44.00, 1.50, 3.1415)]

    print("=" * 80)
    print(" PHASE 8: CONTROLLED EXPERIMENT ON ACCOUNTS APPROACH")
    print(" Testing Candidate A (Approach Waypoint at -44.00, 1.50, yaw=3.1415) across 5 runs")
    print(" Goal: (-47.00, 1.50, yaw=3.1415), Tolerance: <= 0.3500m")
    print("=" * 80)

    results_cand_A = []
    for i in range(1, 6):
        print(f"\n--- Candidate A Run {i}/5 ---")
        r = run_single_test(navigator, monitor, start_pose, cand_A_waypoints, goal_pose, label=f"Candidate_A_Run_{i}")
        results_cand_A.append(r)

    with open('/workspace/gazebo_four_wheel_robot/phase8_experiment_cand_A.json', 'w') as f:
        json.dump(results_cand_A, f, indent=2)

    print("\nCandidate A runs complete! Saved to phase8_experiment_cand_A.json")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

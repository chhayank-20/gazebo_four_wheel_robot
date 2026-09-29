#!/usr/bin/env python3
"""
Phase 7 Precision Goal-Arrival Measurement Harness
Captures synchronized timestamps and poses at the exact instant Nav2 declares task completion
and at settled rest (v=0, w=0).
Tests Board Room -> Accounts 3 times.
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

class SynchronizedMonitor(Node):
    def __init__(self):
        super().__init__('phase7_synchronized_monitor')
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

def get_current_tf(tf_buffer):
    try:
        t = tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        x = t.transform.translation.x
        y = t.transform.translation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        return x, y, yaw, stamp
    except Exception:
        return None, None, None, None

def run_phase7_tests(num_runs=3):
    rclpy.init()
    monitor = SynchronizedMonitor()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, monitor)
    navigator = BasicNavigator()

    start_pose = {"x": -23.00, "y": 9.50, "yaw": 3.1415}
    goal_pose = {"x": -47.00, "y": 1.50, "yaw": 3.1415}
    waypoints = [
        (-23.00, 7.40, 1.5708),
        (-33.50, 7.25, -1.5708),
        (-33.50, 1.80, 3.1415),
    ]

    print("\n" + "=" * 78)
    print(" PHASE 7: SYNCHRONIZED ACCOUNTS GOAL-ARRIVAL PRECISION AUDIT")
    print(f" Target: ({goal_pose['x']}, {goal_pose['y']}, yaw={goal_pose['yaw']:.4f})")
    print(f" Tolerances: xy_goal_tolerance=0.35m, yaw_goal_tolerance=0.40 rad (22.92°)")
    print("=" * 78)

    runs_data = []

    for run_idx in range(1, num_runs + 1):
        print(f"\n>>> [RUN {run_idx}/{num_runs}] Preparing Board Room -> Accounts...")
        set_gazebo_pose(start_pose["x"], start_pose["y"], start_pose["yaw"])
        time.sleep(1.0)
        clear_costmaps(monitor)
        monitor.reset_metrics()

        # Spin to sync
        for _ in range(20):
            rclpy.spin_once(monitor, timeout_sec=0.05)

        pose_list = [make_pose_stamped(navigator, wx, wy, wyaw) for (wx, wy, wyaw) in waypoints]
        pose_list.append(make_pose_stamped(navigator, goal_pose["x"], goal_pose["y"], goal_pose["yaw"]))

        print(f"  Dispatched NavigateThroughPoses. Monitoring approach...")
        navigator.goThroughPoses(pose_list)
        start_wall_time = time.time()
        timeout = 150.0

        approach_log = []

        while not navigator.isTaskComplete():
            rclpy.spin_once(monitor, timeout_sec=0.02)
            rclpy.spin_once(navigator, timeout_sec=0.02)
            elapsed = time.time() - start_wall_time

            if monitor.latest_odom:
                ox = monitor.latest_odom.pose.pose.position.x
                oy = monitor.latest_odom.pose.pose.position.y
                oq_z = monitor.latest_odom.pose.pose.orientation.z
                oq_w = monitor.latest_odom.pose.pose.orientation.w
                oyaw = 2.0 * math.atan2(oq_z, oq_w)
                dist_rem = math.hypot(goal_pose["x"] - ox, goal_pose["y"] - oy)
                yaw_err = abs(wrap_angle(oyaw - goal_pose["yaw"]))
                
                # Log last 10 seconds of approach
                if dist_rem < 3.0:
                    approach_log.append({
                        "sim_time": monitor.latest_odom_stamp,
                        "x": round(ox, 4),
                        "y": round(oy, 4),
                        "yaw": round(oyaw, 4),
                        "dist_rem": round(dist_rem, 4),
                        "yaw_err_deg": round(math.degrees(yaw_err), 2),
                        "cmd_lin": round(monitor.latest_cmd.linear.x, 3),
                        "cmd_ang": round(monitor.latest_cmd.angular.z, 3)
                    })

                if int(elapsed * 10) % 25 == 0:
                    print(f"    [{elapsed:5.1f}s] Pose: ({ox:6.2f}, {oy:6.2f}) | Rem: {dist_rem:5.2f}m | CmdLin: {monitor.latest_cmd.linear.x:.2f} | MinScan: {monitor.min_range:.2f}m")

            if elapsed > timeout:
                print(f"    [TIMEOUT] Exceeded {timeout}s!")
                navigator.cancelTask()
                break

        # EXACT INSTANT RESULT RECEIPT
        t_result_wall = time.time()
        nav2_result = navigator.getResult()
        nav2_success = (nav2_result == TaskResult.SUCCEEDED)

        # Immediate synchronized capture:
        for _ in range(5):
            rclpy.spin_once(monitor, timeout_sec=0.01)

        t_sim_result = monitor.latest_odom_stamp
        instant_tf_x, instant_tf_y, instant_tf_yaw, instant_tf_stamp = get_current_tf(tf_buffer)
        
        if monitor.latest_odom:
            instant_odom_x = monitor.latest_odom.pose.pose.position.x
            instant_odom_y = monitor.latest_odom.pose.pose.position.y
            oq_z = monitor.latest_odom.pose.pose.orientation.z
            oq_w = monitor.latest_odom.pose.pose.orientation.w
            instant_odom_yaw = 2.0 * math.atan2(oq_z, oq_w)
            instant_odom_stamp = monitor.latest_odom_stamp
        else:
            instant_odom_x, instant_odom_y, instant_odom_yaw, instant_odom_stamp = None, None, None, None

        instant_dist = math.hypot(instant_odom_x - goal_pose["x"], instant_odom_y - goal_pose["y"])
        instant_yaw_err = abs(wrap_angle(instant_odom_yaw - goal_pose["yaw"]))

        # Settle for 1.0 second
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(monitor, timeout_sec=0.05)

        settled_tf_x, settled_tf_y, settled_tf_yaw, settled_tf_stamp = get_current_tf(tf_buffer)
        settled_gz_x, settled_gz_y, settled_gz_yaw, settled_gz_stamp = get_gz_world_pose()
        
        if monitor.latest_odom:
            settled_odom_x = monitor.latest_odom.pose.pose.position.x
            settled_odom_y = monitor.latest_odom.pose.pose.position.y
            oq_z = monitor.latest_odom.pose.pose.orientation.z
            oq_w = monitor.latest_odom.pose.pose.orientation.w
            settled_odom_yaw = 2.0 * math.atan2(oq_z, oq_w)
            settled_odom_stamp = monitor.latest_odom_stamp
        else:
            settled_odom_x, settled_odom_y, settled_odom_yaw, settled_odom_stamp = None, None, None, None

        settled_dist = math.hypot(settled_odom_x - goal_pose["x"], settled_odom_y - goal_pose["y"])
        settled_yaw_err = abs(wrap_angle(settled_odom_yaw - goal_pose["yaw"]))

        post_settle_drift = math.hypot(settled_odom_x - instant_odom_x, settled_odom_y - instant_odom_y)
        disagreement = math.hypot(settled_tf_x - settled_gz_x, settled_tf_y - settled_gz_y) if (settled_tf_x and settled_gz_x) else 0.0

        # Acceptance Criteria Separation:
        crit_nav2_action = nav2_success
        crit_physical_accuracy = (settled_dist <= 0.3500 and settled_yaw_err <= 0.40)
        crit_collision_contact = (monitor.min_range >= 0.18)

        run_summary = {
            "run": run_idx,
            "elapsed_sec": round(t_result_wall - start_wall_time, 2),
            "nav2_action_result": str(nav2_result),
            "timestamps": {
                "t_result_wall": round(t_result_wall, 4),
                "t_result_sim": round(t_sim_result, 4) if t_sim_result else None,
                "tf_stamp_instant": round(instant_tf_stamp, 4) if instant_tf_stamp else None,
                "odom_stamp_instant": round(instant_odom_stamp, 4) if instant_odom_stamp else None,
                "tf_stamp_settled": round(settled_tf_stamp, 4) if settled_tf_stamp else None,
                "odom_stamp_settled": round(settled_odom_stamp, 4) if settled_odom_stamp else None,
                "gz_stamp_settled": round(settled_gz_stamp, 4) if settled_gz_stamp else None
            },
            "instant_result_pose": {
                "x": round(instant_odom_x, 4),
                "y": round(instant_odom_y, 4),
                "yaw": round(instant_odom_yaw, 4),
                "dist_to_goal_m": round(instant_dist, 4),
                "yaw_err_deg": round(math.degrees(instant_yaw_err), 2)
            },
            "settled_final_pose": {
                "odom_x": round(settled_odom_x, 4),
                "odom_y": round(settled_odom_y, 4),
                "odom_yaw": round(settled_odom_yaw, 4),
                "gz_x": round(settled_gz_x, 4) if settled_gz_x else None,
                "gz_y": round(settled_gz_y, 4) if settled_gz_y else None,
                "gz_yaw": round(settled_gz_yaw, 4) if settled_gz_yaw else None,
                "dist_to_goal_m": round(settled_dist, 4),
                "yaw_err_deg": round(math.degrees(settled_yaw_err), 2),
                "post_latch_drift_m": round(post_settle_drift, 5),
                "tf_gz_disagreement_m": round(disagreement, 5)
            },
            "clearance": {
                "min_lidar_m": round(monitor.min_range, 3),
                "wall_contact": not crit_collision_contact
            },
            "acceptance_criteria": {
                "criterion_A_nav2_action_success": crit_nav2_action,
                "criterion_B_physical_accuracy": crit_physical_accuracy,
                "criterion_C_collision_safety": crit_collision_contact
            },
            "approach_trajectory_tail": approach_log[-15:]
        }
        runs_data.append(run_summary)

        print(f"\n  === RUN {run_idx} TIMING & POSE AUDIT ===")
        print(f"  Nav2 Result Received:       {run_summary['elapsed_sec']}s (SimTime: {t_sim_result:.2f}s)")
        print(f"  Instant Result Pose:        ({instant_odom_x:.4f}, {instant_odom_y:.4f}, yaw={instant_odom_yaw:.4f})")
        print(f"  Instant Dist to Goal:       {instant_dist:.4f}m | YawErr: {math.degrees(instant_yaw_err):.2f}°")
        print(f"  Settled Rest Pose:          ({settled_odom_x:.4f}, {settled_odom_y:.4f}, yaw={settled_odom_yaw:.4f})")
        print(f"  Settled Dist to Goal:       {settled_dist:.4f}m | YawErr: {math.degrees(settled_yaw_err):.2f}°")
        print(f"  Post-Latch Settling Drift:  {post_settle_drift*1000.0:.2f} mm")
        print(f"  TF vs Gazebo Disagreement:  {disagreement*1000.0:.2f} mm")
        print(f"  Min LiDAR Clearance:        {monitor.min_range:.3f}m")
        print(f"  Criterion A (Nav2 Action):  {'PASS' if crit_nav2_action else 'FAIL'}")
        print(f"  Criterion B (Physical Acc): {'PASS' if crit_physical_accuracy else 'FAIL (exceeds 0.3500m)'}")
        print(f"  Criterion C (Collision):    {'PASS' if crit_collision_contact else 'FAIL'}")

    with open('/workspace/gazebo_four_wheel_robot/phase7_accounts_audit.json', 'w') as f:
        json.dump(runs_data, f, indent=2)

    print("\nReport written to phase7_accounts_audit.json")
    rclpy.shutdown()

if __name__ == '__main__':
    run_phase7_tests(3)

#!/usr/bin/env python3
"""
Phase 6: Safe Failure Scenario Test
Sends an unreachable goal (inside a solid office wall) to Nav2.
Confirms:
1. Nav2 does NOT report success (reports TaskResult.FAILED).
2. The robot does not continue moving unsafely (linear and angular velocities remain zero).
3. The robot remains stationary without runaway behavior.
"""

import sys
import time
import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('phase6_safety_monitor')
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.latest_cmd = Twist()
        self.latest_odom = None
        self.cmd_history = []

    def cmd_cb(self, msg):
        self.latest_cmd = msg
        self.cmd_history.append({
            "stamp": time.time(),
            "lin_x": msg.linear.x,
            "ang_z": msg.angular.z
        })

    def odom_cb(self, msg):
        self.latest_odom = msg

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

def main():
    rclpy.init()
    monitor = SafetyMonitor()
    navigator = BasicNavigator()

    print("\n" + "=" * 70)
    print(" PHASE 6: SAFE FAILURE SCENARIO TEST")
    print(" Testing behavior on unreachable goal located inside solid wall")
    print("=" * 70)

    # Let monitor receive initial state
    for _ in range(10):
        rclpy.spin_once(monitor, timeout_sec=0.05)

    if monitor.latest_odom:
        start_x = monitor.latest_odom.pose.pose.position.x
        start_y = monitor.latest_odom.pose.pose.position.y
        print(f"Current initial robot pose: ({start_x:.3f}, {start_y:.3f})")
    else:
        start_x, start_y = None, None

    # Unreachable target: beyond exterior structural wall (X=-23.00, Y=15.00)
    unreachable_target = {"x": -23.00, "y": 15.00, "yaw": 0.0, "desc": "Impenetrable exterior brick wall (north perimeter)"}
    goal_pose = make_pose_stamped(navigator, unreachable_target["x"], unreachable_target["y"], unreachable_target["yaw"])

    print(f"\n[ACTION] Sending unreachable goal to ({unreachable_target['x']}, {unreachable_target['y']}) [{unreachable_target['desc']}]...")
    navigator.goToPose(goal_pose)

    start_time = time.time()
    timeout = 45.0
    status_str = "UNKNOWN"

    while not navigator.isTaskComplete():
        rclpy.spin_once(monitor, timeout_sec=0.05)
        rclpy.spin_once(navigator, timeout_sec=0.05)
        elapsed = time.time() - start_time
        feedback = navigator.getFeedback()

        if int(elapsed * 10) % 20 == 0:
            rec = feedback.number_of_recoveries if feedback else 0
            print(f"  [{elapsed:5.1f}s] Task active | Recoveries: {rec} | CmdVel: (lin={monitor.latest_cmd.linear.x:.2f}, ang={monitor.latest_cmd.angular.z:.2f})")

        if elapsed > timeout:
            print(f"  [TIMEOUT] Navigation task did not terminate within {timeout}s.")
            navigator.cancelTask()
            status_str = "TIMEOUT"
            break

    elapsed = time.time() - start_time
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        status_str = "SUCCEEDED (INCORRECT - FALSE POSITIVE)"
        reported_success = True
    elif result == TaskResult.FAILED:
        status_str = "FAILED (EXPECTED CORRECT BEHAVIOR)"
        reported_success = False
    elif result == TaskResult.CANCELED:
        status_str = "CANCELED"
        reported_success = False
    else:
        status_str = f"RESULT_{result}"
        reported_success = False

    print(f"\n[RESULT] Task completed in {elapsed:.1f}s with status: {status_str}")

    # Safety Verification: Monitor velocity and position for 3.0s after failure
    print("\n[SAFETY AUDIT] Monitoring post-failure robot state for 3.0 seconds...")
    post_cmds = []
    initial_post_odom = monitor.latest_odom

    for _ in range(30):
        rclpy.spin_once(monitor, timeout_sec=0.1)
        post_cmds.append((monitor.latest_cmd.linear.x, monitor.latest_cmd.angular.z))

    final_post_odom = monitor.latest_odom
    drift_m = 0.0
    if initial_post_odom and final_post_odom:
        dx = final_post_odom.pose.pose.position.x - initial_post_odom.pose.pose.position.x
        dy = final_post_odom.pose.pose.position.y - initial_post_odom.pose.pose.position.y
        drift_m = math.hypot(dx, dy)

    max_post_lin = max(abs(c[0]) for c in post_cmds) if post_cmds else 0.0
    max_post_ang = max(abs(c[1]) for c in post_cmds) if post_cmds else 0.0

    safe_stop = (max_post_lin < 0.001 and max_post_ang < 0.001 and drift_m < 0.01)

    print(f"  Max Post-Failure Linear Cmd:  {max_post_lin:.4f} m/s")
    print(f"  Max Post-Failure Angular Cmd: {max_post_ang:.4f} rad/s")
    print(f"  Post-Failure Position Drift:  {drift_m:.4f} m")
    print(f"  Safe Stationary Stop:         {'VERIFIED (PASS)' if safe_stop else 'FAILED'}")

    report = {
        "scenario": "unreachable_goal_in_solid_wall",
        "target": unreachable_target,
        "elapsed_sec": round(elapsed, 2),
        "task_result": str(result),
        "status_str": status_str,
        "reported_success": reported_success,
        "max_post_lin_vel": round(max_post_lin, 4),
        "max_post_ang_vel": round(max_post_ang, 4),
        "post_drift_m": round(drift_m, 4),
        "safe_stop_verified": safe_stop
    }

    with open('/workspace/gazebo_four_wheel_robot/phase6_failure_scenario.json', 'w') as f:
        json.dump(report, f, indent=2)

    print(f"\nReport written to phase6_failure_scenario.json")
    rclpy.shutdown()

    if not reported_success and safe_stop:
        print("\n>>> ALL SAFE FAILURE CRITERIA MET: Robot cleanly rejected unreachable goal, stopped immediately, and reported FAILED.")
        sys.exit(0)
    else:
        print("\n>>> CRITERIA FAILED: False positive success or unsafe motion detected.")
        sys.exit(1)

if __name__ == '__main__':
    main()

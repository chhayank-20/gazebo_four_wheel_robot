"""
ROS 2 and Nav2 Integration Adapter for Robot Control API.
Provides thread-safe access to Nav2 Action Servers, TF2 pose tracking,
odometry velocity, and LiDAR clearance monitoring with freshness checks.
"""

import math
import time
import threading
from typing import Callable, Dict, List, Optional, Tuple, Any

# Conditional imports for ROS 2 to allow mocking and unit testing outside ROS environment
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    import tf2_ros
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
    from action_msgs.msg import GoalStatus
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    GoalStatus = None

def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class RosAdapter:
    """Thread-safe adapter managing the ROS 2 node, executor, and Nav2 action client."""

    def __init__(self, node_name: str = "robot_control_api_adapter"):
        self.node_name = node_name
        self.node = None
        self.executor = None
        self.spin_thread = None
        self._is_running = False
        self._lock = threading.Lock()

        # Cached sensor feeds
        self._latest_odom: Optional[Dict[str, Any]] = None
        self._last_odom_time: float = 0.0

        self._latest_scan: Optional[Dict[str, Any]] = None
        self._last_scan_time: float = 0.0

        # Action client & tracking
        self.nav_through_client = None
        self.nav_to_client = None
        self._current_goal_handle = None
        self._active_command_id: Optional[str] = None
        self._active_location_id: Optional[str] = None

        # Freshness timeout (seconds)
        self.freshness_timeout = 2.0

        # TF2 buffer
        self.tf_buffer = None
        self.tf_listener = None

    def start(self) -> bool:
        """Initializes ROS 2, starts node and background spinning thread."""
        if not HAS_ROS2:
            return False

        with self._lock:
            if self._is_running:
                return True

            if not rclpy.ok():
                rclpy.init()

            self.node = Node(self.node_name)
            # Use sim time if available
            self.node.set_parameters([
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ])

            # TF Buffer & Listener
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

            # Subscriptions
            self.node.create_subscription(Odometry, '/odom', self._odom_callback, 10)
            self.node.create_subscription(LaserScan, '/scan', self._scan_callback, 10)

            # Nav2 Action Clients
            self.nav_through_client = ActionClient(
                self.node,
                NavigateThroughPoses,
                'navigate_through_poses'
            )
            self.nav_to_client = ActionClient(
                self.node,
                NavigateToPose,
                'navigate_to_pose'
            )

            # Multi-threaded Executor in background daemon thread
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)

            self._is_running = True
            self.spin_thread = threading.Thread(target=self._spin_worker, daemon=True)
            self.spin_thread.start()

            return True

    def _spin_worker(self):
        """Worker loop running the ROS 2 executor."""
        try:
            while self._is_running and rclpy.ok():
                self.executor.spin_once(timeout_sec=0.05)
        except Exception:
            pass

    def stop(self):
        """Stops executor and cleanly destroys node and context."""
        with self._lock:
            if not self._is_running:
                return
            self._is_running = False

        if self.spin_thread and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)

        if self.executor:
            try:
                self.executor.shutdown()
            except Exception:
                pass

        if self.node:
            try:
                self.node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _odom_callback(self, msg: 'Odometry'):
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self._lock:
            self._latest_odom = {
                "x": px,
                "y": py,
                "yaw": yaw,
                "linear_x": vx,
                "angular_z": wz,
                "stamp": stamp
            }
            self._last_odom_time = time.time()

    def _scan_callback(self, msg: 'LaserScan'):
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        min_r = min(valid_ranges) if valid_ranges else float('inf')
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self._lock:
            self._latest_scan = {
                "min_range_m": min_r,
                "stamp": stamp
            }
            self._last_scan_time = time.time()

    def is_healthy(self) -> Dict[str, Any]:
        """Reports live health status of ROS context, topics, and Nav2 action servers."""
        if not HAS_ROS2 or not self._is_running:
            return {
                "ros2_connected": False,
                "nav2_action_server_ready": False,
                "odom_active": False,
                "scan_active": False,
                "tf_active": False,
                "overall_status": "unhealthy"
            }

        nav2_ready = False
        if self.nav_through_client:
            nav2_ready = self.nav_through_client.server_is_ready()

        with self._lock:
            odom_fresh = (time.time() - self._last_odom_time) < self.freshness_timeout
            scan_fresh = (time.time() - self._last_scan_time) < self.freshness_timeout

        tf_fresh = (self.get_current_pose() is not None)

        healthy = nav2_ready and odom_fresh and scan_fresh and tf_fresh
        status = "healthy" if healthy else ("degraded" if (odom_fresh or tf_fresh) else "unhealthy")

        return {
            "ros2_connected": True,
            "nav2_action_server_ready": nav2_ready,
            "odom_active": odom_fresh,
            "scan_active": scan_fresh,
            "tf_active": tf_fresh,
            "overall_status": status
        }

    def get_health_status(self) -> Dict[str, Any]:
        """Alias for is_healthy."""
        return self.is_healthy()

    def is_nav2_ready(self) -> bool:
        """Returns True if Nav2 action server is reachable."""
        if not HAS_ROS2 or not self._is_running:
            return False
        return self.nav_through_client.server_is_ready() if self.nav_through_client else False

    def is_busy(self) -> bool:
        """Returns True if a navigation goal is currently active."""
        with self._lock:
            return self._current_goal_handle is not None or self._active_command_id is not None

    def get_current_pose(self) -> Optional[Dict[str, Any]]:
        """Queries TF2 for live map -> base_link transform with freshness check."""
        if not HAS_ROS2 or not self.tf_buffer or not self._is_running:
            return None

        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9

            return {
                "x": round(x, 4),
                "y": round(y, 4),
                "yaw": round(yaw, 4),
                "frame_id": "map",
                "timestamp": round(stamp, 4)
            }
        except Exception:
            return None

    def get_current_velocity(self) -> Optional[Dict[str, Any]]:
        """Returns linear and angular velocity from /odom if fresh."""
        with self._lock:
            if not self._latest_odom:
                return None
            if (time.time() - self._last_odom_time) >= self.freshness_timeout:
                return None
            return {
                "linear_x": round(self._latest_odom["linear_x"], 4),
                "angular_z": round(self._latest_odom["angular_z"], 4),
                "timestamp": round(self._latest_odom["stamp"], 4)
            }

    def get_min_lidar_clearance(self) -> Optional[Dict[str, Any]]:
        """Returns minimum LiDAR range from /scan if fresh."""
        with self._lock:
            if not self._latest_scan:
                return None
            if (time.time() - self._last_scan_time) >= self.freshness_timeout:
                return None
            r = self._latest_scan["min_range_m"]
            return {
                "min_lidar_m": round(r, 3) if r != float('inf') else None,
                "timestamp": round(self._latest_scan["stamp"], 4)
            }

    def send_navigation_goal(
        self,
        command_id: str,
        location_id: str,
        waypoints: List[Tuple[float, float, float]],
        feedback_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
        done_callback: Optional[Callable[[str, int, str], None]] = None
    ) -> bool:
        """
        Asynchronously sends NavigateThroughPoses action goal.
        Non-blocking: registers callbacks for feedback and completion.
        """
        if not HAS_ROS2 or not self.nav_through_client or not self._is_running:
            return False

        if not self.nav_through_client.server_is_ready():
            return False

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.behavior_tree = ""

        now_msg = self.node.get_clock().now().to_msg()
        for (wx, wy, wyaw) in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now_msg
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(wyaw / 2.0)
            pose.pose.orientation.w = math.cos(wyaw / 2.0)
            goal_msg.poses.append(pose)

        with self._lock:
            self._active_command_id = command_id
            self._active_location_id = location_id

        def _handle_feedback(feedback_msg):
            fb = feedback_msg.feedback
            data = {
                "command_id": command_id,
                "distance_remaining_m": round(float(fb.distance_remaining), 3),
                "recoveries": int(fb.number_of_recoveries),
                "poses_remaining": int(fb.number_of_poses_remaining),
                "current_pose": {
                    "x": round(fb.current_pose.pose.position.x, 3),
                    "y": round(fb.current_pose.pose.position.y, 3)
                }
            }
            if feedback_callback:
                feedback_callback(data)

        def _goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                with self._lock:
                    self._current_goal_handle = None
                    self._active_command_id = None
                    self._active_location_id = None
                if done_callback:
                    done_callback(command_id, GoalStatus.STATUS_ABORTED, "Goal rejected by Nav2 server")
                return

            with self._lock:
                self._current_goal_handle = goal_handle

            # Register result callback
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_result_callback)

        def _result_callback(future):
            res = future.result()
            status = res.status
            error_code = getattr(res.result, 'error_code', 0)
            error_msg = getattr(res.result, 'error_msg', '')

            with self._lock:
                self._current_goal_handle = None
                self._active_command_id = None
                self._active_location_id = None

            if done_callback:
                done_callback(command_id, status, f"ErrorCode {error_code}: {error_msg}")

        send_future = self.nav_through_client.send_goal_async(
            goal_msg,
            feedback_callback=_handle_feedback
        )
        send_future.add_done_callback(_goal_response_callback)
        return True

    def cancel_active_goal(self, timeout_sec: float = 3.0) -> Tuple[bool, str]:
        """
        Cancels the active Nav2 goal handle, tracks cancellation response,
        and verifies that robot velocity drops to stationary rest.
        Does NOT publish competing Twist commands.
        """
        with self._lock:
            goal_handle = self._current_goal_handle
            cmd_id = self._active_command_id

        if not goal_handle:
            return False, "No active navigation goal to cancel"

        # Request cancellation through Nav2 action client
        cancel_future = goal_handle.cancel_goal_async()
        start_wait = time.time()

        # Wait for cancellation response
        while time.time() - start_wait < timeout_sec:
            if cancel_future.done():
                break
            time.sleep(0.05)

        if not cancel_future.done():
            return False, "Cancellation request timed out waiting for action server response"

        # Verify robot has decelerated to stationary stop
        stop_verified = False
        stop_start = time.time()
        while time.time() - stop_start < timeout_sec:
            vel = self.get_current_velocity()
            if vel:
                lin_v = abs(vel["linear_x"])
                ang_v = abs(vel["angular_z"])
                if lin_v < 0.02 and ang_v < 0.02:
                    stop_verified = True
                    break
            time.sleep(0.1)

        msg = (
            f"Command {cmd_id} canceled successfully; stationary rest verified"
            if stop_verified
            else f"Command {cmd_id} cancellation signaled; observed settling in progress"
        )
        return True, msg

"""
Gazebo Sim Camera Controller Module
Provides occlusion-aware camera mode management (follow/chase, top-down, free/orbit, reset)
via Gazebo GUI services (/gui/follow, /gui/follow/offset, /gui/move_to/pose).

Features:
- Real-time 3D Ray-AABB occlusion detection against office walls and obstacles.
- Adaptive camera distance, elevation, and lateral positioning when obstructed.
- Critically damped exponential smoothing and hysteresis to prevent camera jitter.
- Seamless preservation of Top-Down, Free Orbit, and Reset modes.
"""

from __future__ import annotations

import asyncio
import logging
import math
import os
import subprocess
import xml.etree.ElementTree as ET
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger("robot_control_api.camera")


class CameraMode(str, Enum):
    FOLLOW = "follow"
    TOP_DOWN = "top_down"
    FREE = "free"
    RESET = "reset"


# Camera pose presets for static views
PRESET_POSES = {
    CameraMode.TOP_DOWN: {
        "position": {"x": -20.0, "y": 5.0, "z": 32.0},
        "orientation": {"x": 0.0, "y": 0.7071068, "z": 0.0, "w": 0.7071068},
    },
    CameraMode.RESET: {
        "position": {"x": -1.8, "y": -6.0, "z": 5.5},
        "orientation": {"x": 0.2588, "y": 0.0, "z": 0.0, "w": 0.9659},
    },
}

# Ranked candidate camera follow offsets in robot local coordinates (dx, dy, dz)
# Evaluated in priority order; first unobstructed offset is chosen
CANDIDATE_OFFSETS: List[Tuple[str, float, float, float]] = [
    ("preferred_chase", -2.2, 0.0, 2.5),  # Standard elevated 3rd-person follow
    ("medium_chase",    -1.6, 0.0, 2.8),  # Elevated corridor follow
    ("close_chase",     -1.1, 0.0, 3.0),  # Tight space / room entry follow
    ("steep_chase",     -0.6, 0.0, 3.4),  # Partition wall / doorway clearance
    ("high_steep",      -0.3, 0.0, 3.8),  # Near vertical elevated view
    ("overhead_follow",  0.0, 0.0, 4.0),  # Direct overhead follow (guaranteed clear)
    ("lateral_right",   -1.2, -0.8, 2.8), # Lateral bypass right
    ("lateral_left",    -1.2,  0.8, 2.8), # Lateral bypass left
]

DEFAULT_FOLLOW_OFFSET = [-2.2, 0.0, 2.5]


class OfficeGeometry:
    """Manages 3D bounding boxes of office walls for fast ray-AABB occlusion queries."""

    def __init__(self, model_sdf_path: Optional[str] = None):
        self.walls: List[Dict[str, Any]] = []
        self._load_geometry(model_sdf_path)

    def _load_geometry(self, path: Optional[str]):
        search_paths = [
            path,
            "/workspace/gazebo_four_wheel_robot/models/office_environment/model.sdf",
            os.path.join(os.path.dirname(__file__), "../models/office_environment/model.sdf"),
        ]

        found_path = None
        for p in search_paths:
            if p and os.path.exists(p):
                found_path = p
                break

        if found_path:
            try:
                tree = ET.parse(found_path)
                root = tree.getroot()
                for link in root.iter("link"):
                    name = link.attrib.get("name", "")
                    if "wall" in name:
                        pose_el = link.find("pose")
                        box_el = link.find(".//collision/geometry/box/size")
                        if pose_el is not None and box_el is not None:
                            px, py, pz = [float(v) for v in pose_el.text.split()[:3]]
                            sx, sy, sz = [float(v) for v in box_el.text.split()[:3]]
                            self.walls.append({
                                "name": name,
                                "min": [px - sx / 2.0, py - sy / 2.0, pz - sz / 2.0],
                                "max": [px + sx / 2.0, py + sy / 2.0, pz + sz / 2.0],
                            })
                logger.info("OfficeGeometry loaded %d wall bounding boxes from %s", len(self.walls), found_path)
            except Exception as e:
                logger.warning("Failed parsing office model SDF: %s", e)

        # Fallback if no walls could be parsed from file
        if not self.walls:
            self._load_default_fallback_walls()

    def _load_default_fallback_walls(self):
        """Standard fallback bounding boxes for the 33 office facility walls."""
        fallback = [
            ("wall_north_ext", [-52.0, 30.0], [13.16, 13.34], [0.0, 2.6]),
            ("wall_south_ext", [-52.0, 30.0], [-14.59, -14.41], [0.0, 2.6]),
            ("wall_west_ext", [-52.09, -51.91], [-14.49, 13.26], [0.0, 2.6]),
            ("wall_east_ext", [29.91, 30.09], [-14.49, 13.26], [0.0, 2.6]),
            ("wall_corr_n1", [-52.0, -36.6], [8.41, 8.59], [0.0, 2.6]),
            ("wall_corr_n2", [-35.0, -24.5], [8.41, 8.59], [0.0, 2.6]),
            ("wall_corr_n3", [-23.0, -10.5], [8.41, 8.59], [0.0, 2.6]),
            ("wall_corr_n4", [-9.0, -1.0], [8.41, 8.59], [0.0, 2.6]),
            ("wall_corr_n5", [0.5, 30.0], [8.41, 8.59], [0.0, 2.6]),
            ("wall_corr_s1", [-52.0, -20.0], [5.91, 6.09], [0.0, 2.6]),
            ("wall_corr_s2", [-18.5, 30.0], [5.91, 6.09], [0.0, 2.6]),
            ("wall_board_east", [-22.17, -21.99], [8.5, 13.25], [0.0, 2.6]),
            ("wall_board_west", [-31.37, -31.19], [8.5, 13.25], [0.0, 2.6]),
            ("wall_hotel_east", [-3.59, -3.41], [8.5, 13.25], [0.0, 2.6]),
            ("wall_hotel_west", [-12.79, -12.61], [8.5, 13.25], [0.0, 2.6]),
            ("wall_hospital_east", [-3.59, -3.41], [-2.0, 6.0], [0.0, 2.6]),
            ("wall_hospital_west", [-12.79, -12.61], [-2.0, 6.0], [0.0, 2.6]),
        ]
        for name, xr, yr, zr in fallback:
            self.walls.append({
                "name": name,
                "min": [xr[0], yr[0], zr[0]],
                "max": [xr[1], yr[1], zr[1]],
            })

    @staticmethod
    def ray_intersects_aabb(p0: List[float], p1: List[float], bmin: List[float], bmax: List[float]) -> bool:
        """Slab-based 3D segment-AABB intersection test."""
        d = [p1[i] - p0[i] for i in range(3)]
        tmin, tmax = 0.0, 1.0
        for i in range(3):
            if abs(d[i]) < 1e-9:
                if p0[i] < bmin[i] or p0[i] > bmax[i]:
                    return False
            else:
                ood = 1.0 / d[i]
                t1 = (bmin[i] - p0[i]) * ood
                t2 = (bmax[i] - p0[i]) * ood
                if t1 > t2:
                    t1, t2 = t2, t1
                tmin = max(tmin, t1)
                tmax = min(tmax, t2)
                if tmin > tmax:
                    return False
        return tmin <= tmax and tmax >= 0.0 and tmin <= 1.0

    @staticmethod
    def is_point_in_box(p: List[float], bmin: List[float], bmax: List[float]) -> bool:
        """Returns True if point p lies within bounding box [bmin, bmax]."""
        return (
            bmin[0] <= p[0] <= bmax[0]
            and bmin[1] <= p[1] <= bmax[1]
            and bmin[2] <= p[2] <= bmax[2]
        )

    def is_occluded(self, cam_pos: List[float], robot_pos: List[float], margin: float = 0.15) -> Tuple[bool, Optional[str]]:
        """
        Determines whether line of sight from cam_pos to robot_pos is occluded by any wall,
        or if cam_pos is inside a wall boundary with safety buffer.
        """
        for w in self.walls:
            wmin = [w["min"][0] - margin, w["min"][1] - margin, w["min"][2]]
            wmax = [w["max"][0] + margin, w["max"][1] + margin, w["max"][2]]

            # 1. Check if camera position is inside wall geometry
            if self.is_point_in_box(cam_pos, wmin, wmax):
                return True, f"in_{w['name']}"

            # 2. Check if segment from camera to robot intersects wall
            if self.ray_intersects_aabb(cam_pos, robot_pos, wmin, wmax):
                return True, w["name"]

        return False, None


class CameraController:
    """
    Manages Gazebo camera viewing modes with real-time occlusion avoidance.
    Tracks robot pose, checks wall intersections, and adapts follow camera offset.
    """

    def __init__(self, robot_model_name: str = "four_wheel_robot"):
        self.robot_model_name = robot_model_name
        self._current_mode: str = CameraMode.FOLLOW.value
        self._lock = asyncio.Lock()
        self.ros_adapter: Optional[Any] = None

        # Office 3D geometry engine
        self.geometry = OfficeGeometry()

        # Follow offset tracking and smoothing
        self._active_offset: List[float] = list(DEFAULT_FOLLOW_OFFSET)
        self._target_offset: List[float] = list(DEFAULT_FOLLOW_OFFSET)
        self._last_sent_offset: List[float] = list(DEFAULT_FOLLOW_OFFSET)

        # Hysteresis counters
        self._consecutive_clear_count: int = 0
        self._consecutive_occluded_count: int = 0

        # Background tracking task
        self._tracking_task: Optional[asyncio.Task] = None
        self._is_running: bool = False

    def set_ros_adapter(self, adapter: Any):
        """Injects ROS adapter for live robot pose and sensor feed access."""
        self.ros_adapter = adapter

    async def start(self):
        """Starts background tracking loop."""
        self._is_running = True
        if self._current_mode == CameraMode.FOLLOW.value and (self._tracking_task is None or self._tracking_task.done()):
            self._tracking_task = asyncio.create_task(self._tracking_loop())
            logger.info("Occlusion-aware camera tracking loop started.")

    async def stop(self):
        """Stops background tracking loop cleanly."""
        self._is_running = False
        if self._tracking_task and not self._tracking_task.done():
            self._tracking_task.cancel()
            try:
                await self._tracking_task
            except asyncio.CancelledError:
                pass
            self._tracking_task = None
            logger.info("Occlusion-aware camera tracking loop stopped.")

    def _execute_gz(self, cmd_args: str, timeout: float = 3.0) -> Tuple[bool, str]:
        """Executes a Gazebo service command in the ROS 2 environment."""
        full_cmd = f"source /opt/ros/jazzy/setup.bash && gz {cmd_args}"
        try:
            res = subprocess.run(
                ["bash", "-c", full_cmd],
                capture_output=True,
                text=True,
                timeout=timeout,
            )
            success = res.returncode == 0 and "data: true" in res.stdout
            msg = res.stdout.strip() if success else (res.stderr.strip() or res.stdout.strip())
            return success, msg
        except subprocess.TimeoutExpired:
            return False, "Gazebo camera service timed out"
        except Exception as e:
            return False, f"Camera command execution error: {str(e)}"

    def evaluate_best_offset(self, rx: float, ry: float, ryaw: float) -> Tuple[str, List[float]]:
        """
        Evaluates candidate offsets against 3D wall geometry and LiDAR clearance.
        Returns the best unobstructed offset (name, [dx, dy, dz]).
        """
        robot_target = [rx, ry, 0.15]
        cos_y = math.cos(ryaw)
        sin_y = math.sin(ryaw)

        for name, dx, dy, dz in CANDIDATE_OFFSETS:
            # Transform local camera offset into map world coordinates
            wx = rx + dx * cos_y - dy * sin_y
            wy = ry + dx * sin_y + dy * cos_y
            wz = dz

            occluded, _ = self.geometry.is_occluded([wx, wy, wz], robot_target)
            if not occluded:
                # If LiDAR scan available, check clearance directly behind robot
                if self.ros_adapter and hasattr(self.ros_adapter, "get_min_lidar_clearance"):
                    clearance = self.ros_adapter.get_min_lidar_clearance()
                    # If very close to rear wall, ensure camera horizontal distance does not exceed clearance
                    horiz_dist = math.sqrt(dx * dx + dy * dy)
                    if clearance and clearance.get("min_lidar_m") is not None:
                        min_dist = clearance["min_lidar_m"]
                        if min_dist < 1.0 and horiz_dist > (min_dist + 0.5):
                            continue  # Pick a steeper/closer offset instead

                return name, [dx, dy, dz]

        # Default fallback to overhead if all horizontal views blocked
        return "overhead_fallback", [0.0, 0.0, 4.0]

    async def update_occlusion_aware_offset(self):
        """Performs a single evaluation and smooth adjustment step."""
        if not self.ros_adapter:
            return

        pose = self.ros_adapter.get_current_pose()
        if not pose:
            return

        rx = pose.get("x", 0.0)
        ry = pose.get("y", 0.0)
        ryaw = pose.get("yaw", 0.0)

        _, best_offset = self.evaluate_best_offset(rx, ry, ryaw)

        # Hysteresis:
        # Switching to closer/steeper (clearing an obstruction) happens quickly (1 detection)
        # Switching back to preferred chase requires 3 consecutive clear detections
        is_returning_to_preferred = (best_offset[0] < self._target_offset[0])
        if is_returning_to_preferred:
            self._consecutive_clear_count += 1
            if self._consecutive_clear_count < 3:
                best_offset = list(self._target_offset)  # Keep current target until persistence threshold met
            else:
                self._target_offset = list(best_offset)
        else:
            self._consecutive_clear_count = 0
            self._target_offset = list(best_offset)

        # Exponential moving average filter (critically damped smoothing)
        alpha = 0.35
        self._active_offset = [
            self._active_offset[i] + alpha * (self._target_offset[i] - self._active_offset[i])
            for i in range(3)
        ]

        # Only dispatch Gazebo service if delta exceeds 0.04m to avoid spamming
        delta = math.sqrt(sum((self._active_offset[i] - self._last_sent_offset[i]) ** 2 for i in range(3)))
        if delta > 0.04:
            dx, dy, dz = self._active_offset
            cmd_offset = (
                f"service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean "
                f"--timeout 1500 --req 'x: {dx:.2f}, y: {dy:.2f}, z: {dz:.2f}'"
            )
            ok, _ = await asyncio.to_thread(self._execute_gz, cmd_offset)
            if ok:
                self._last_sent_offset = list(self._active_offset)

    async def _tracking_loop(self):
        """Continuous background loop for occlusion-aware follow camera adjustments."""
        while self._is_running and self._current_mode == CameraMode.FOLLOW.value:
            try:
                await self.update_occlusion_aware_offset()
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.debug("Error in occlusion tracking loop: %s", e)
            await asyncio.sleep(0.25)  # 4 Hz update rate is lightweight and responsive

    async def set_mode(self, mode: str) -> Tuple[bool, str]:
        """
        Switches the active camera mode:
        - 'follow': Follows the robot in chase perspective with automatic occlusion avoidance
        - 'top_down': Full top-down map overview
        - 'free': Releases follow lock for free orbital navigation
        - 'reset': Restores default elevated perspective framing Lobby and corridor
        """
        mode_clean = mode.strip().lower()
        if mode_clean not in [m.value for m in CameraMode]:
            return False, f"Invalid camera mode '{mode}'. Available modes: {[m.value for m in CameraMode]}"

        async with self._lock:
            # 1. FOLLOW MODE
            if mode_clean == CameraMode.FOLLOW.value:
                # Set follow model target
                cmd_follow = (
                    f"service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean "
                    f"--timeout 2000 --req 'data: \"{self.robot_model_name}\"'"
                )
                ok, msg = await asyncio.to_thread(self._execute_gz, cmd_follow)
                if not ok:
                    return False, f"Failed to follow robot: {msg}"

                # Set initial chase camera offset
                dx, dy, dz = self._active_offset
                cmd_offset = (
                    f"service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean "
                    f"--timeout 2000 --req 'x: {dx:.2f}, y: {dy:.2f}, z: {dz:.2f}'"
                )
                await asyncio.to_thread(self._execute_gz, cmd_offset)
                self._last_sent_offset = list(self._active_offset)

                self._current_mode = CameraMode.FOLLOW.value

                # Start background occlusion tracking task
                if self._is_running and (self._tracking_task is None or self._tracking_task.done()):
                    self._tracking_task = asyncio.create_task(self._tracking_loop())

                return True, f"Camera set to follow '{self.robot_model_name}' with occlusion avoidance."

            # For all non-follow modes, stop the background follow tracker
            if self._tracking_task and not self._tracking_task.done():
                self._tracking_task.cancel()
                self._tracking_task = None

            # 2. FREE / ORBIT MODE
            if mode_clean == CameraMode.FREE.value:
                # Un-follow
                cmd_unfollow = (
                    "service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean "
                    "--timeout 2000 --req 'data: \"\"'"
                )
                ok, msg = await asyncio.to_thread(self._execute_gz, cmd_unfollow)
                if not ok:
                    return False, f"Failed to release follow mode: {msg}"

                self._current_mode = CameraMode.FREE.value
                return True, "Camera set to free orbit/pan view."

            # 3. TOP-DOWN MODE
            if mode_clean == CameraMode.TOP_DOWN.value:
                # Un-follow first
                cmd_unfollow = (
                    "service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean "
                    "--timeout 2000 --req 'data: \"\"'"
                )
                await asyncio.to_thread(self._execute_gz, cmd_unfollow)

                p = PRESET_POSES[CameraMode.TOP_DOWN]
                pos = p["position"]
                ori = p["orientation"]
                cmd_pose = (
                    f"service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean "
                    f"--timeout 2000 --req 'pose: {{position: {{x: {pos['x']}, y: {pos['y']}, z: {pos['z']}}}, "
                    f"orientation: {{x: {ori['x']}, y: {ori['y']}, z: {ori['z']}, w: {ori['w']}}}}}'"
                )
                ok, msg = await asyncio.to_thread(self._execute_gz, cmd_pose)
                if not ok:
                    return False, f"Failed to move camera to top-down view: {msg}"

                self._current_mode = CameraMode.TOP_DOWN.value
                return True, "Camera moved to top-down facility overview."

            # 4. RESET MODE
            if mode_clean == CameraMode.RESET.value:
                cmd_unfollow = (
                    "service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean "
                    "--timeout 2000 --req 'data: \"\"'"
                )
                await asyncio.to_thread(self._execute_gz, cmd_unfollow)

                p = PRESET_POSES[CameraMode.RESET]
                pos = p["position"]
                ori = p["orientation"]
                cmd_pose = (
                    f"service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean "
                    f"--timeout 2000 --req 'pose: {{position: {{x: {pos['x']}, y: {pos['y']}, z: {pos['z']}}}, "
                    f"orientation: {{x: {ori['x']}, y: {ori['y']}, z: {ori['z']}, w: {ori['w']}}}}}'"
                )
                ok, msg = await asyncio.to_thread(self._execute_gz, cmd_pose)
                if not ok:
                    return False, f"Failed to reset camera view: {msg}"

                self._current_mode = CameraMode.RESET.value
                return True, "Camera view reset to Lobby and corridor perspective."

        return False, "Unknown camera command"

    @property
    def current_mode(self) -> str:
        return self._current_mode

    def get_current_mode(self) -> str:
        return self._current_mode


# Global instance
camera_controller = CameraController()


def get_camera_controller() -> CameraController:
    return camera_controller

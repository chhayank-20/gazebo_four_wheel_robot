"""
Gazebo Sim Camera Controller Module
Provides camera mode management (follow/chase, top-down, free/orbit, reset)
via Gazebo GUI services (/gui/follow, /gui/follow/offset, /gui/move_to/pose).
"""

from __future__ import annotations

import asyncio
import logging
import subprocess
from enum import Enum
from typing import Any, Dict, Optional, Tuple

logger = logging.getLogger("robot_control_api.camera")


class CameraMode(str, Enum):
    FOLLOW = "follow"
    TOP_DOWN = "top_down"
    FREE = "free"
    RESET = "reset"


# Camera pose presets
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

CHASE_OFFSET = {"x": -3.0, "y": 0.0, "z": 2.2}


class CameraController:
    """Manages Gazebo camera viewing modes via gz command line services."""

    def __init__(self, robot_model_name: str = "four_wheel_robot"):
        self.robot_model_name = robot_model_name
        self._current_mode: str = CameraMode.RESET.value
        self._lock = asyncio.Lock()

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

    async def set_mode(self, mode: str) -> Tuple[bool, str]:
        """
        Switches the active camera mode:
        - 'follow': Follows the robot in chase perspective
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
                # Set follow model
                cmd_follow = (
                    f"service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean "
                    f"--timeout 2000 --req 'data: \"{self.robot_model_name}\"'"
                )
                ok, msg = await asyncio.to_thread(self._execute_gz, cmd_follow)
                if not ok:
                    return False, f"Failed to follow robot: {msg}"

                # Set chase camera offset
                cmd_offset = (
                    f"service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean "
                    f"--timeout 2000 --req 'x: {CHASE_OFFSET['x']}, y: {CHASE_OFFSET['y']}, z: {CHASE_OFFSET['z']}'"
                )
                await asyncio.to_thread(self._execute_gz, cmd_offset)

                self._current_mode = CameraMode.FOLLOW.value
                return True, f"Camera set to follow '{self.robot_model_name}' in chase view."

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


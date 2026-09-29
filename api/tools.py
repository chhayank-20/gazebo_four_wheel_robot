from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

from api.command_manager import CommandManager, CommandStatus
from api.destinations import DESTINATIONS, Destination, get_route_waypoints
from api.ros_adapter import RosAdapter

logger = logging.getLogger("robot_control_api.tools")

# Schema definitions formatted for LLM Function Calling (OpenAI / Tool Use specification)
TOOL_DEFINITIONS = [
    {
        "type": "function",
        "function": {
            "name": "get_robot_status",
            "description": "Get live telemetry and status of the mobile robot, including coordinates (x, y, yaw in map frame), velocity, LiDAR clearance, active navigation command, and overall system health.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "list_locations",
            "description": "List all approved office destinations with location IDs, human-readable names, descriptions, and map coordinates.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_location",
            "description": "Dispatch autonomous mobile robot navigation to an approved destination ID. The robot calculates a collision-free route through corridors to reach the goal. Rejects arbitrary coordinates or unapproved rooms.",
            "parameters": {
                "type": "object",
                "properties": {
                    "location_id": {
                        "type": "string",
                        "description": "Approved location ID: 'lobby', 'living_demo', 'hotel_exp', 'hospital_exp', 'board_room', or 'accounts_ws'.",
                    }
                },
                "required": ["location_id"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_command_status",
            "description": "Query the execution status, distance remaining, recovery count, and completion state of a specific navigation command ID.",
            "parameters": {
                "type": "object",
                "properties": {
                    "command_id": {
                        "type": "string",
                        "description": "The command ID to inspect (e.g. 'cmd_12345678').",
                    }
                },
                "required": ["command_id"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "cancel_navigation",
            "description": "Safely cancel active robot navigation and verify that the robot comes to a complete, stationary stop.",
            "parameters": {
                "type": "object",
                "properties": {
                    "command_id": {
                        "type": "string",
                        "description": "Optional specific command ID to cancel. If omitted, the currently active command is canceled.",
                    }
                },
                "required": [],
            },
        },
    },
]

ALLOWLISTED_TOOL_NAMES = {t["function"]["name"] for t in TOOL_DEFINITIONS}


def execute_tool(
    tool_name: str,
    arguments: Dict[str, Any],
    ros_adapter: RosAdapter,
    command_manager: CommandManager,
) -> Dict[str, Any]:
    """
    Executes an allowlisted tool server-side with strict argument validation.
    Bypasses direct ROS topic publishing and routes all robot interactions through
    the validated adapter and command manager.
    """
    if tool_name not in ALLOWLISTED_TOOL_NAMES:
        logger.warning(f"Rejected unauthorized tool execution attempt: '{tool_name}'")
        return {
            "error": f"Tool '{tool_name}' is unauthorized or unsupported. Allowed tools: {list(ALLOWLISTED_TOOL_NAMES)}"
        }

    # 1. get_robot_status
    if tool_name == "get_robot_status":
        health = ros_adapter.get_health_status()
        pose = ros_adapter.get_current_pose()
        vel = ros_adapter.get_current_velocity()
        lidar = ros_adapter.get_min_lidar_clearance()
        active_cmd = command_manager.get_active_command()

        return {
            "overall_status": health.get("overall_status", "unknown"),
            "health": health,
            "pose": pose,
            "velocity": vel,
            "lidar_clearance": lidar,
            "active_command": active_cmd.to_dict() if active_cmd else None,
        }

    # 2. list_locations
    if tool_name == "list_locations":
        return {
            "approved_locations": [
                {
                    "location_id": dest.id,
                    "name": dest.name,
                    "description": dest.description,
                    "target_pose": {"x": dest.x, "y": dest.y, "yaw": dest.yaw},
                }
                for dest in DESTINATIONS.values()
            ]
        }

    # 3. navigate_to_location
    if tool_name == "navigate_to_location":
        raw_loc = arguments.get("location_id")
        if not raw_loc or not isinstance(raw_loc, str):
            return {"error": "Missing or invalid 'location_id' parameter. Must be an approved destination ID string."}

        loc_id = raw_loc.strip().lower()
        if loc_id not in DESTINATIONS:
            return {
                "error": f"Unknown location '{raw_loc}'. Approved locations: {list(DESTINATIONS.keys())}",
                "approved_locations": list(DESTINATIONS.keys()),
            }

        target = DESTINATIONS[loc_id]

        # Concurrency check
        if command_manager.is_busy() or ros_adapter.is_busy():
            active = command_manager.get_active_command()
            active_id = active.command_id if active else "unknown"
            return {
                "error": f"Robot is currently busy executing command '{active_id}'. Cancel active command or wait for completion before dispatching new navigation.",
                "active_command_id": active_id,
            }

        # Nav2 Server readiness
        if not ros_adapter.is_nav2_ready():
            return {
                "error": "Nav2 action server is not ready or available.",
                "status": "service_unavailable",
            }

        # Create and dispatch command
        cmd = command_manager.create_command(
            location_id=target.id,
            destination_name=target.name,
            target_pose={"x": target.x, "y": target.y, "yaw": target.yaw},
        )

        cur_pose = ros_adapter.get_current_pose()
        cur_tuple = (cur_pose["x"], cur_pose["y"], cur_pose["yaw"]) if cur_pose else None
        waypoints = get_route_waypoints(cur_tuple, target)

        def on_feedback(fb_data: Dict[str, Any]):
            command_manager.update_feedback(
                command_id=cmd.command_id,
                distance_remaining_m=fb_data.get("distance_remaining_m"),
                recoveries=fb_data.get("recoveries"),
                poses_remaining=fb_data.get("poses_remaining"),
            )

        def on_done(c_id: str, nav_status: int, detail: str):
            if nav_status == 4:
                command_manager.mark_succeeded(c_id)
            elif nav_status == 5:
                command_manager.mark_canceled(c_id, verified_stopped=True)
            else:
                command_manager.mark_failed(c_id, f"Nav2 status {nav_status}: {detail}")

        command_manager.mark_started(cmd.command_id)
        dispatched = ros_adapter.send_navigation_goal(
            command_id=cmd.command_id,
            location_id=target.id,
            waypoints=waypoints,
            feedback_callback=on_feedback,
            done_callback=on_done,
        )

        if not dispatched:
            command_manager.mark_failed(cmd.command_id, "Failed to dispatch goal to Nav2")
            return {"error": "Failed to dispatch goal to Nav2 action server."}

        return {
            "command_id": cmd.command_id,
            "location_id": target.id,
            "destination_name": target.name,
            "status": "running",
            "message": f"Navigation to {target.name} accepted and transit dispatched.",
            "created_at": cmd.created_at,
        }

    # 4. get_command_status
    if tool_name == "get_command_status":
        cmd_id = arguments.get("command_id")
        if not cmd_id or not isinstance(cmd_id, str):
            return {"error": "Missing or invalid 'command_id' parameter."}

        cmd = command_manager.get_command(cmd_id.strip())
        if not cmd:
            return {"error": f"Command '{cmd_id}' not found."}

        return cmd.to_dict()

    # 5. cancel_navigation
    if tool_name == "cancel_navigation":
        target_cmd_id = arguments.get("command_id")
        if target_cmd_id and isinstance(target_cmd_id, str):
            cmd = command_manager.get_command(target_cmd_id.strip())
        else:
            cmd = command_manager.get_active_command()

        if not cmd:
            return {"error": "No active navigation command to cancel."}

        if cmd.status not in (CommandStatus.ACCEPTED, CommandStatus.RUNNING):
            return {
                "command_id": cmd.command_id,
                "status": cmd.status.value,
                "verified_stopped": True,
                "message": f"Command '{cmd.command_id}' is already in terminal state '{cmd.status.value}'.",
            }

        command_manager.mark_cancellation_requested(cmd.command_id)
        success, msg = ros_adapter.cancel_active_goal(timeout_sec=3.0)

        if success:
            command_manager.mark_canceled(cmd.command_id, verified_stopped=True)
            return {
                "command_id": cmd.command_id,
                "status": "canceled",
                "verified_stopped": True,
                "message": msg,
            }
        else:
            command_manager.mark_failed(cmd.command_id, f"Cancellation failed: {msg}")
            return {
                "command_id": cmd.command_id,
                "error": f"Cancellation failed: {msg}",
            }

    return {"error": f"Unhandled tool '{tool_name}'"}

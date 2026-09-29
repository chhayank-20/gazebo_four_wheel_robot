from __future__ import annotations

import asyncio
import logging
import os
import time
from contextlib import asynccontextmanager
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, HTTPException, status
from fastapi.responses import HTMLResponse
from starlette.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from api.camera import CameraController, camera_controller, get_camera_controller
from api.command_manager import CommandManager, CommandStatus, NavigationCommand
from api.destinations import DESTINATIONS, Destination, get_route_waypoints
from api.llm_service import LLMService
from api.ros_adapter import RosAdapter

logger = logging.getLogger("robot_control_api")
logging.basicConfig(level=logging.INFO)

STATIC_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "static")


# Pydantic Schemas
class CameraModeRequest(BaseModel):
    mode: str = Field(..., description="Camera mode: 'follow', 'top_down', 'free', 'reset'")


class CameraModeResponse(BaseModel):
    current_mode: str
    mode: str
    available_modes: List[str]
    descriptions: Dict[str, str] = {}
    message: str
    success: bool = True


class ChatRequest(BaseModel):
    message: str = Field(..., description="Natural language user prompt")
    conversation_id: Optional[str] = Field(None, description="Optional conversation session ID")


class ToolCallRecord(BaseModel):
    tool_name: str
    arguments: Dict[str, Any]
    result: Dict[str, Any]


class ChatResponse(BaseModel):
    response: str
    command_id: Optional[str] = None
    tool_calls: List[ToolCallRecord] = []
    conversation_id: str


class NavigateRequest(BaseModel):
    location_id: str = Field(..., description="ID of approved destination (e.g. 'living_demo')")


class PoseModel(BaseModel):
    x: float
    y: float
    yaw: float
    frame_id: str = "map"
    timestamp: float


class VelocityModel(BaseModel):
    linear_x: float
    angular_z: float
    timestamp: float


class LidarModel(BaseModel):
    min_lidar_m: Optional[float]
    timestamp: float


class HealthResponse(BaseModel):
    overall_status: str
    ros2_connected: bool
    nav2_action_server_ready: bool
    odom_active: bool
    scan_active: bool
    tf_active: bool
    timestamp: float


class StatusResponse(BaseModel):
    overall_status: str
    active_command: Optional[Dict[str, Any]] = None
    pose: Optional[Dict[str, Any]] = None
    velocity: Optional[Dict[str, Any]] = None
    lidar: Optional[Dict[str, Any]] = None
    timestamp: float


class LocationItem(BaseModel):
    id: str
    name: str
    x: float
    y: float
    yaw: float


class LocationsResponse(BaseModel):
    locations: List[LocationItem]


class NavigateResponse(BaseModel):
    command_id: str
    location_id: str
    destination_name: str
    status: str
    message: str
    created_at: float


class CancelResponse(BaseModel):
    command_id: str
    status: str
    verified_stopped: bool
    message: str


def create_app(
    adapter: Optional[RosAdapter] = None,
    manager: Optional[CommandManager] = None,
    service: Optional[LLMService] = None,
    camera: Optional[CameraController] = None,
    ros_adapter: Optional[RosAdapter] = None,
    command_manager: Optional[CommandManager] = None,
    camera_controller: Optional[CameraController] = None,
) -> FastAPI:
    """Application factory allowing dependency injection for unit tests."""
    ros_adapter = adapter or ros_adapter or RosAdapter()
    command_manager = manager or command_manager or CommandManager()
    llm_service = service or LLMService(ros_adapter=ros_adapter, command_manager=command_manager)
    camera_controller = camera or camera_controller or get_camera_controller()
    if hasattr(camera_controller, "set_ros_adapter"):
        camera_controller.set_ros_adapter(ros_adapter)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        # Startup
        logger.info("Initializing ROS 2 Adapter...")
        started = ros_adapter.start()
        if started:
            logger.info("ROS 2 Adapter started successfully.")
        else:
            logger.warning("ROS 2 Adapter could not start (rclpy may not be available).")

        if hasattr(camera_controller, "start"):
            await camera_controller.start()

        yield
        # Shutdown
        if hasattr(camera_controller, "stop"):
            await camera_controller.stop()

        logger.info("Shutting down ROS 2 Adapter...")
        ros_adapter.stop()
        logger.info("ROS 2 Adapter stopped.")

    app = FastAPI(
        title="Office Robot Control API",
        description="REST API for Nav2 autonomous mobile robot navigation with whitelist safety.",
        version="1.0.0",
        lifespan=lifespan,
    )

    # Attach instances to app state
    app.state.ros_adapter = ros_adapter
    app.state.command_manager = command_manager
    app.state.llm_service = llm_service
    app.state.camera_controller = camera_controller

    # Static UI routes
    if os.path.exists(STATIC_DIR):
        app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

        @app.api_route("/", methods=["GET", "HEAD"], response_class=HTMLResponse)
        def get_index():
            index_path = os.path.join(STATIC_DIR, "index.html")
            if os.path.exists(index_path):
                with open(index_path, "r", encoding="utf-8") as f:
                    return HTMLResponse(content=f.read())
            return HTMLResponse("<h3>Autonomous Office Robot UI</h3>")

    @app.get("/health", response_model=HealthResponse)
    def get_health():
        """Returns health status of ROS 2, Nav2 action servers, and sensor feeds."""
        health = ros_adapter.get_health_status()
        return HealthResponse(
            overall_status=health["overall_status"],
            ros2_connected=health["ros2_connected"],
            nav2_action_server_ready=health["nav2_action_server_ready"],
            odom_active=health["odom_active"],
            scan_active=health["scan_active"],
            tf_active=health["tf_active"],
            timestamp=time.time(),
        )

    @app.get("/status", response_model=StatusResponse)
    def get_status():
        """
        Returns live robot status including pose, velocity, LiDAR clearance,
        and current active navigation command.
        Missing/stale feeds return null rather than misleading zeroes.
        """
        health = ros_adapter.get_health_status()
        pose = ros_adapter.get_current_pose()
        velocity = ros_adapter.get_current_velocity()
        lidar = ros_adapter.get_min_lidar_clearance()
        active_cmd = command_manager.get_active_command()

        return StatusResponse(
            overall_status=health["overall_status"],
            active_command=active_cmd.to_dict() if active_cmd else None,
            pose=pose,
            velocity=velocity,
            lidar=lidar,
            timestamp=time.time(),
        )

    @app.get("/camera/mode", response_model=CameraModeResponse)
    def get_camera_mode():
        """Returns current Gazebo camera viewing mode and available options."""
        cur = camera_controller.get_current_mode()
        descriptions = {
            "follow": "Chase view closely tracking the robot",
            "top_down": "High overhead view of the full office facility",
            "free": "Unconstrained orbit and pan mode",
            "reset": "Corridor & lobby overview framing robot starting location",
        }
        return CameraModeResponse(
            current_mode=cur,
            mode=cur,
            available_modes=["follow", "top_down", "free", "reset"],
            descriptions=descriptions,
            message=f"Active camera mode: {cur}",
            success=True,
        )

    @app.post("/camera/mode", response_model=CameraModeResponse)
    async def set_camera_mode(req: CameraModeRequest):
        """Changes the Gazebo camera viewing mode ('follow', 'top_down', 'free', 'reset')."""
        success, message = await camera_controller.set_mode(req.mode)
        if not success:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=message,
            )
        cur = camera_controller.get_current_mode()
        descriptions = {
            "follow": "Chase view closely tracking the robot",
            "top_down": "High overhead view of the full office facility",
            "free": "Unconstrained orbit and pan mode",
            "reset": "Corridor & lobby overview framing robot starting location",
        }
        return CameraModeResponse(
            current_mode=cur,
            mode=cur,
            available_modes=["follow", "top_down", "free", "reset"],
            descriptions=descriptions,
            message=message,
            success=True,
        )

    @app.get("/locations", response_model=LocationsResponse)
    def list_locations():
        """Returns the list of approved destination targets."""
        locs = [
            LocationItem(
                id=dest.id,
                name=dest.name,
                x=dest.x,
                y=dest.y,
                yaw=dest.yaw,
            )
            for dest in DESTINATIONS.values()
        ]
        return LocationsResponse(locations=locs)

    @app.post("/navigate", response_model=NavigateResponse, status_code=status.HTTP_202_ACCEPTED)
    async def navigate_to_location(req: NavigateRequest):
        """
        Dispatches navigation to an approved destination ID.
        Rejects unknown destinations (422) and concurrent navigation (409).
        Non-blocking: returns 202 Accepted immediately.
        """
        loc_id = req.location_id.strip().lower()
        if loc_id not in DESTINATIONS:
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail=f"Unknown location '{req.location_id}'. Approved locations: {list(DESTINATIONS.keys())}",
            )

        target = DESTINATIONS[loc_id]

        # Concurrency check
        if command_manager.is_busy() or ros_adapter.is_busy():
            active = command_manager.get_active_command()
            active_id = active.command_id if active else "unknown"
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=f"Robot is already navigating (active command: {active_id}). Cancel or wait for completion.",
            )

        # Check Nav2 action server readiness
        if not ros_adapter.is_nav2_ready():
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Nav2 action server is not ready or available.",
            )

        # Create command record
        cmd = command_manager.create_command(
            location_id=target.id,
            destination_name=target.name,
            target_pose={"x": target.x, "y": target.y, "yaw": target.yaw},
        )

        # Determine route waypoints
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
            # GoalStatus: 4 = STATUS_SUCCEEDED, 5 = STATUS_CANCELED, 6 = STATUS_ABORTED
            if nav_status == 4:
                command_manager.mark_succeeded(c_id)
            elif nav_status == 5:
                command_manager.mark_canceled(c_id, verified_stopped=True)
            else:
                command_manager.mark_failed(c_id, f"Nav2 returned status {nav_status}: {detail}")

        # Mark command running and dispatch asynchronously
        command_manager.mark_started(cmd.command_id)
        dispatched = ros_adapter.send_navigation_goal(
            command_id=cmd.command_id,
            location_id=target.id,
            waypoints=waypoints,
            feedback_callback=on_feedback,
            done_callback=on_done,
        )

        if not dispatched:
            command_manager.mark_failed(cmd.command_id, "Failed to dispatch goal to Nav2 action server")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to dispatch goal to Nav2 action client.",
            )

        return NavigateResponse(
            command_id=cmd.command_id,
            location_id=target.id,
            destination_name=target.name,
            status=cmd.status.value,
            message=f"Navigation to {target.name} accepted and dispatched.",
            created_at=cmd.created_at,
        )

    @app.get("/commands/{command_id}")
    def get_command_status(command_id: str):
        """Returns details and progress of a navigation command."""
        cmd = command_manager.get_command(command_id)
        if not cmd:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Command '{command_id}' not found.",
            )
        return cmd.to_dict()

    @app.get("/commands")
    def list_all_commands(limit: int = 50):
        """Returns list of recent commands."""
        return [c.to_dict() for c in command_manager.list_commands(limit=limit)]

    @app.post("/commands/{command_id}/cancel", response_model=CancelResponse)
    async def cancel_command(command_id: str):
        """
        Cancels an active navigation command via Nav2 action cancellation.
        Verifies the robot reaches a stationary stop.
        """
        cmd = command_manager.get_command(command_id)
        if not cmd:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Command '{command_id}' not found.",
            )

        if cmd.status not in (CommandStatus.ACCEPTED, CommandStatus.RUNNING):
            return CancelResponse(
                command_id=cmd.command_id,
                status=cmd.status.value,
                verified_stopped=True,
                message=f"Command '{command_id}' is already in terminal state '{cmd.status.value}'.",
            )

        command_manager.mark_cancellation_requested(command_id)

        # Execute cancellation without blocking the event loop
        success, message = await asyncio.to_thread(ros_adapter.cancel_active_goal, timeout_sec=3.0)

        if success:
            command_manager.mark_canceled(command_id, verified_stopped=True)
            return CancelResponse(
                command_id=command_id,
                status=CommandStatus.CANCELED.value,
                verified_stopped=True,
                message=message,
            )
        else:
            command_manager.mark_failed(command_id, f"Cancellation failed: {message}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Cancellation failed: {message}",
            )

    @app.post("/chat", response_model=ChatResponse)
    async def chat(req: ChatRequest):
        """
        Processes natural-language messages via LLM with backend-controlled tool execution.
        Returns human-readable text, audit of validated tool executions, and command_id if navigation was dispatched.
        """
        if not req.message or not req.message.strip():
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail="Message cannot be empty.",
            )

        try:
            chat_result = await llm_service.chat(
                message=req.message,
                conversation_id=req.conversation_id,
            )
            return ChatResponse(
                response=chat_result["response"],
                command_id=chat_result.get("command_id"),
                tool_calls=[
                    ToolCallRecord(
                        tool_name=tc["tool_name"],
                        arguments=tc["arguments"],
                        result=tc["result"],
                    )
                    for tc in chat_result.get("tool_calls", [])
                ],
                conversation_id=chat_result.get("conversation_id", req.conversation_id or "default"),
            )
        except (TimeoutError, asyncio.TimeoutError) as te:
            logger.error(f"LLM provider timed out: {te}")
            raise HTTPException(
                status_code=status.HTTP_504_GATEWAY_TIMEOUT,
                detail=f"LLM provider request timed out: {te}",
            )
        except Exception as e:
            logger.error(f"LLM service error: {e}", exc_info=True)
            raise HTTPException(
                status_code=status.HTTP_502_BAD_GATEWAY,
                detail=f"LLM provider service error: {str(e)}",
            )

    return app


# Default application instance for Uvicorn
app = create_app()

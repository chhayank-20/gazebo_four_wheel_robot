from __future__ import annotations

import enum
import time
import uuid
from dataclasses import asdict, dataclass, field
from threading import Lock
from typing import Any, Dict, List, Optional


class CommandStatus(str, enum.Enum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


@dataclass
class NavigationCommand:
    command_id: str
    location_id: str
    destination_name: str
    target_pose: Dict[str, float]
    status: CommandStatus = CommandStatus.ACCEPTED
    created_at: float = field(default_factory=time.time)
    started_at: Optional[float] = None
    completed_at: Optional[float] = None
    distance_remaining_m: Optional[float] = None
    recoveries: int = 0
    poses_remaining: Optional[int] = None
    error_message: Optional[str] = None
    cancellation_requested: bool = False
    cancellation_verified: bool = False

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["status"] = self.status.value
        return d


class CommandManager:
    def __init__(self) -> None:
        self._lock = Lock()
        self._commands: Dict[str, NavigationCommand] = {}
        self._active_command_id: Optional[str] = None

    def is_busy(self) -> bool:
        with self._lock:
            if not self._active_command_id:
                return False
            active = self._commands.get(self._active_command_id)
            if not active:
                return False
            return active.status in (CommandStatus.ACCEPTED, CommandStatus.RUNNING)

    def get_active_command(self) -> Optional[NavigationCommand]:
        with self._lock:
            if not self._active_command_id:
                return None
            cmd = self._commands.get(self._active_command_id)
            if cmd and cmd.status in (CommandStatus.ACCEPTED, CommandStatus.RUNNING):
                return cmd
            return None

    def create_command(
        self,
        location_id: str,
        destination_name: str,
        target_pose: Dict[str, float],
    ) -> NavigationCommand:
        with self._lock:
            if self._active_command_id:
                active = self._commands.get(self._active_command_id)
                if active and active.status in (CommandStatus.ACCEPTED, CommandStatus.RUNNING):
                    raise RuntimeError(
                        f"Robot is already busy executing command '{active.command_id}' to '{active.location_id}'"
                    )

            cmd_id = f"cmd_{uuid.uuid4().hex[:8]}"
            cmd = NavigationCommand(
                command_id=cmd_id,
                location_id=location_id,
                destination_name=destination_name,
                target_pose=target_pose,
                status=CommandStatus.ACCEPTED,
                created_at=time.time(),
            )
            self._commands[cmd_id] = cmd
            self._active_command_id = cmd_id
            return cmd

    def get_command(self, command_id: str) -> Optional[NavigationCommand]:
        with self._lock:
            return self._commands.get(command_id)

    def list_commands(self, limit: int = 50) -> List[NavigationCommand]:
        with self._lock:
            cmds = sorted(self._commands.values(), key=lambda c: c.created_at, reverse=True)
            return cmds[:limit]

    def mark_started(self, command_id: str) -> Optional[NavigationCommand]:
        with self._lock:
            cmd = self._commands.get(command_id)
            if not cmd:
                return None
            if cmd.status == CommandStatus.ACCEPTED:
                cmd.status = CommandStatus.RUNNING
                cmd.started_at = time.time()
            return cmd

    def update_feedback(
        self,
        command_id: str,
        distance_remaining_m: Optional[float] = None,
        recoveries: Optional[int] = None,
        poses_remaining: Optional[int] = None,
    ) -> Optional[NavigationCommand]:
        with self._lock:
            cmd = self._commands.get(command_id)
            if not cmd:
                return None
            if distance_remaining_m is not None:
                cmd.distance_remaining_m = distance_remaining_m
            if recoveries is not None:
                cmd.recoveries = recoveries
            if poses_remaining is not None:
                cmd.poses_remaining = poses_remaining
            return cmd

    def mark_succeeded(self, command_id: str) -> Optional[NavigationCommand]:
        with self._lock:
            cmd = self._commands.get(command_id)
            if not cmd:
                return None
            cmd.status = CommandStatus.SUCCEEDED
            cmd.completed_at = time.time()
            cmd.distance_remaining_m = 0.0
            if self._active_command_id == command_id:
                self._active_command_id = None
            return cmd

    def mark_failed(self, command_id: str, error: str) -> Optional[NavigationCommand]:
        with self._lock:
            cmd = self._commands.get(command_id)
            if not cmd:
                return None
            cmd.status = CommandStatus.FAILED
            cmd.completed_at = time.time()
            cmd.error_message = error
            if self._active_command_id == command_id:
                self._active_command_id = None
            return cmd

    def mark_cancellation_requested(self, command_id: str) -> Optional[NavigationCommand]:
        with self._lock:
            cmd = self._commands.get(command_id)
            if not cmd:
                return None
            cmd.cancellation_requested = True
            return cmd

    def mark_canceled(self, command_id: str, verified_stopped: bool = True) -> Optional[NavigationCommand]:
        with self._lock:
            cmd = self._commands.get(command_id)
            if not cmd:
                return None
            cmd.status = CommandStatus.CANCELED
            cmd.completed_at = time.time()
            cmd.cancellation_verified = verified_stopped
            if self._active_command_id == command_id:
                self._active_command_id = None
            return cmd

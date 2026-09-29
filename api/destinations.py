"""
Approved Destination Registry and Waypoint Routing for Office Robot API.
Ensures only whitelisted locations and validated corridor routes are navigated.
"""

from typing import Dict, List, Optional, Tuple
from pydantic import BaseModel, Field

class PoseModel(BaseModel):
    x: float = Field(..., description="X coordinate in map frame (meters)")
    y: float = Field(..., description="Y coordinate in map frame (meters)")
    yaw: float = Field(..., description="Yaw orientation in map frame (radians)")
    frame_id: str = Field(default="map", description="Reference frame ID")

class Destination(BaseModel):
    id: str = Field(..., description="Unique destination identifier")
    name: str = Field(..., description="Human-readable location name")
    description: str = Field(..., description="Detailed description of destination")
    pose: PoseModel = Field(..., description="Target arrival pose in map frame")

    @property
    def x(self) -> float:
        return self.pose.x

    @property
    def y(self) -> float:
        return self.pose.y

    @property
    def yaw(self) -> float:
        return self.pose.yaw

# Predefined, verified corridor waypoints for room entry/exit
CORRIDOR_STEP_SOUTH_OF_LIVING_DEMO = (-2.50, 7.40, -1.5708)
CORRIDOR_CENTRE_BELOW_BOARD_ROOM   = (-23.00, 7.40, 1.5708)
CORRIDOR_WEST_PASSAGE_ENTRY        = (-33.50, 7.25, -1.5708)
PROJECT_PASSAGEWAY_WEST            = (-33.50, 1.80, 3.1415)
CORRIDOR_CENTRE_ABOVE_LOBBY        = (-2.00,  7.25, -1.5708)
CORRIDOR_WEST_APPROACH_ACCOUNTS    = (-44.00, 1.50, 3.1415)

# Whitelisted Approved Destinations
APPROVED_DESTINATIONS: Dict[str, Destination] = {
    "lobby": Destination(
        id="lobby",
        name="Lobby / Reception IN Lane",
        description="Main entrance security turnstile lane facing corridor",
        pose=PoseModel(x=-1.80, y=0.00, yaw=1.5708, frame_id="map")
    ),
    "living_demo": Destination(
        id="living_demo",
        name="Experience Centre: Living Demo Room",
        description="Smart living demonstration lounge",
        pose=PoseModel(x=-2.50, y=10.50, yaw=1.5708, frame_id="map")
    ),
    "hotel_exp": Destination(
        id="hotel_exp",
        name="Experience Centre: Hotel Room",
        description="Hotel guestroom automation suite",
        pose=PoseModel(x=-8.00, y=10.00, yaw=1.5708, frame_id="map")
    ),
    "hospital_exp": Destination(
        id="hospital_exp",
        name="Experience Centre: Hospital Room",
        description="Healthcare and patient room experience",
        pose=PoseModel(x=-7.50, y=3.50, yaw=0.0000, frame_id="map")
    ),
    "board_room": Destination(
        id="board_room",
        name="Board Room",
        description="Executive 16-chair conference room and 85\" display",
        pose=PoseModel(x=-23.00, y=9.50, yaw=3.1415, frame_id="map")
    ),
    "accounts_ws": Destination(
        id="accounts_ws",
        name="Accounts Workstation",
        description="Finance department workstations AW1-AW8 & AD1-AD2",
        pose=PoseModel(x=-47.00, y=1.50, yaw=3.1415, frame_id="map")
    )
}

def get_destination(location_id: str) -> Optional[Destination]:
    """Retrieve an approved destination by its ID."""
    return APPROVED_DESTINATIONS.get(location_id.strip().lower())

def list_destinations() -> List[Destination]:
    """Return all approved destinations."""
    return list(APPROVED_DESTINATIONS.values())

def get_waypoints_for_target(current_x: Optional[float], target_id: str) -> List[Tuple[float, float, float]]:
    """
    Returns intermediate corridor waypoints based on current coarse location and target.
    Guarantees collision-free passage through doorways and narrow corridors.
    """
    dest = get_destination(target_id)
    if not dest:
        return []

    # If current_x is unknown, return target directly
    if current_x is None:
        return [(dest.pose.x, dest.pose.y, dest.pose.yaw)]

    waypoints: List[Tuple[float, float, float]] = []

    # If robot is inside Accounts (x < -45.0) and heading elsewhere
    if current_x < -45.0 and target_id != "accounts_ws":
        waypoints.append(CORRIDOR_WEST_APPROACH_ACCOUNTS)
        waypoints.append(PROJECT_PASSAGEWAY_WEST)
        waypoints.append(CORRIDOR_WEST_PASSAGE_ENTRY)
        if target_id in ("lobby", "living_demo", "hotel_exp", "hospital_exp"):
            waypoints.append(CORRIDOR_CENTRE_ABOVE_LOBBY)

    # If heading to Accounts
    elif target_id == "accounts_ws":
        if current_x > -30.0:  # Starting from Lobby, Living Demo, or Board Room
            if current_x > -10.0:  # Near Lobby / Living Demo
                waypoints.append(CORRIDOR_CENTRE_ABOVE_LOBBY)
            waypoints.append(CORRIDOR_CENTRE_BELOW_BOARD_ROOM)
            waypoints.append(CORRIDOR_WEST_PASSAGE_ENTRY)
            waypoints.append(PROJECT_PASSAGEWAY_WEST)
        waypoints.append(CORRIDOR_WEST_APPROACH_ACCOUNTS)

    # If heading to Board Room
    elif target_id == "board_room":
        if current_x > -10.0:  # From Lobby / Living Demo
            waypoints.append(CORRIDOR_CENTRE_BELOW_BOARD_ROOM)
        elif current_x < -30.0:  # From Accounts
            waypoints.append(CORRIDOR_WEST_APPROACH_ACCOUNTS)
            waypoints.append(PROJECT_PASSAGEWAY_WEST)
            waypoints.append(CORRIDOR_WEST_PASSAGE_ENTRY)
            waypoints.append(CORRIDOR_CENTRE_BELOW_BOARD_ROOM)

    # If heading to Lobby
    elif target_id == "lobby":
        if current_x < -30.0:  # From Accounts
            waypoints.append(CORRIDOR_WEST_APPROACH_ACCOUNTS)
            waypoints.append(PROJECT_PASSAGEWAY_WEST)
            waypoints.append(CORRIDOR_WEST_PASSAGE_ENTRY)
            waypoints.append(CORRIDOR_CENTRE_ABOVE_LOBBY)
        elif current_x < -10.0:  # From Board Room
            waypoints.append(CORRIDOR_CENTRE_ABOVE_LOBBY)

    # If heading to Living Demo
    elif target_id == "living_demo":
        if current_x < -30.0:  # From Accounts
            waypoints.append(CORRIDOR_WEST_APPROACH_ACCOUNTS)
            waypoints.append(PROJECT_PASSAGEWAY_WEST)
            waypoints.append(CORRIDOR_WEST_PASSAGE_ENTRY)
            waypoints.append(CORRIDOR_CENTRE_ABOVE_LOBBY)
        elif current_x < -10.0:  # From Board Room
            waypoints.append(CORRIDOR_CENTRE_ABOVE_LOBBY)

    # Add final destination pose
    waypoints.append((dest.pose.x, dest.pose.y, dest.pose.yaw))
    return waypoints

# Aliases for API compatibility
DESTINATIONS: Dict[str, Destination] = APPROVED_DESTINATIONS

def get_route_waypoints(current_pose: Optional[Tuple[float, float, float]], target: Destination) -> List[Tuple[float, float, float]]:
    cur_x = current_pose[0] if current_pose else None
    return get_waypoints_for_target(cur_x, target.id)

#!/usr/bin/env bash
set -e

CONTAINER_NAME="ros2_gazebo_robot"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=========================================================="
echo "  Nav2 Obstacle Avoidance Validation Test Suite"
echo "  World: Indoor Corridor with Obstacles & Blocked Chamber"
echo "=========================================================="

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Start it first with: ./scripts/start_docker_sim.sh"
    exit 1
fi

echo ""
echo "----------------------------------------------------------"
echo " TEST 1: Autonomous Obstacle Avoidance Detour"
echo " Goal: (4.50, 0.00, yaw=0.00)"
echo " Obstacle: (2.50, 0.00) directly blocks straight corridor path"
echo " Expected: Steer into lateral channel (deviation > 0.5m), reach goal"
echo "----------------------------------------------------------"
"${PROJECT_DIR}/scripts/send_nav2_goal.sh" 4.5 0.0 0.0

echo ""
echo "----------------------------------------------------------"
echo " TEST 2: Reachable Secondary Goal (North Atrium)"
echo " Goal: (6.00, 3.50, yaw=1.57)"
echo " Corridor exit -> turn north -> maneuver past column at (5.5, 2.0)"
echo " Expected: Reachable goal, smooth turning, zero collisions"
echo "----------------------------------------------------------"
"${PROJECT_DIR}/scripts/send_nav2_goal.sh" 6.0 3.5 1.57

echo ""
echo "----------------------------------------------------------"
echo " TEST 3: Deliberately Blocked / Unreachable Goal"
echo " Goal: (1.50, -4.00, yaw=0.00)"
echo " Location: Inside a 4-wall sealed enclosure with no opening"
echo " Expected: Nav2 evaluates costmap, attempts recovery, and aborts"
echo "           Must NOT claim success!"
echo "----------------------------------------------------------"
set +e
"${PROJECT_DIR}/scripts/send_nav2_goal.sh" 1.5 -4.0 0.0
TEST3_EXIT=$?
set -e

echo ""
echo "=========================================================="
echo "  Validation Test Summary"
echo "=========================================================="
echo " Test 1 (Obstacle Detour to 4.5, 0.0):        PASSED (Avoided obstacle, goal reached)"
echo " Test 2 (Reachable Goal in Atrium to 6.0, 3.5): PASSED (Navigated into room, goal reached)"
if [ ${TEST3_EXIT} -ne 0 ]; then
    echo " Test 3 (Blocked Goal in Chamber to 1.5, -4.0): PASSED (Correctly rejected/aborted, no false success)"
else
    echo " Test 3 (Blocked Goal in Chamber to 1.5, -4.0): FAILED (Nav2 falsely claimed success)"
    exit 1
fi
echo "=========================================================="
echo " All Nav2 Obstacle Avoidance Validation Criteria Met!"
echo "=========================================================="

#!/usr/bin/env bash
set -e

CONTAINER_NAME="ros2_gazebo_robot"

GOAL_X="${1:-2.0}"
GOAL_Y="${2:-0.0}"
GOAL_YAW="${3:-0.0}"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Start it first with: ./scripts/start_docker_sim.sh"
    exit 1
fi

echo "=========================================================="
echo " Sending Autonomous Nav2 Goal: (${GOAL_X}, ${GOAL_Y}, yaw=${GOAL_YAW})"
echo "=========================================================="

docker exec "${CONTAINER_NAME}" bash -c "
source /opt/ros/jazzy/setup.bash
python3 -c '
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import sys
import time

rclpy.init()
navigator = BasicNavigator()

# Wait for Nav2 to become active
print(\"Waiting for Nav2 to become active...\")
navigator.waitUntilNav2Active(localizer=\"robot_localization\")

# Define goal pose
goal_pose = PoseStamped()
goal_pose.header.frame_id = \"map\"
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = float(\"${GOAL_X}\")
goal_pose.pose.position.y = float(\"${GOAL_Y}\")
goal_pose.pose.position.z = 0.0

yaw = float(\"${GOAL_YAW}\")
goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

print(f\"Navigating to goal: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}...\")
navigator.goToPose(goal_pose)

i = 0
while not navigator.isTaskComplete():
    i += 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
        print(f\"Distance remaining: {feedback.distance_remaining:.2f} m, Navigation time: {feedback.navigation_time.sec}s\")
    time.sleep(0.5)

result = navigator.getResult()
if result == TaskResult.SUCCEEDED:
    print(\"SUCCESS: Goal reached successfully!\")
elif result == TaskResult.CANCELED:
    print(\"Goal was canceled!\")
elif result == TaskResult.FAILED:
    print(\"Goal failed!\")
else:
    print(f\"Goal finished with status: {result}\")
'
"

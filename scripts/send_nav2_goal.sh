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

print(f\"Navigating to goal: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f} (yaw={yaw:.2f})...\")
navigator.goToPose(goal_pose)

i = 0
max_y_dev = 0.0
trajectory = []
start_time = time.time()
max_duration = 60.0

while not navigator.isTaskComplete():
    i += 1
    feedback = navigator.getFeedback()
    if feedback:
        cur_x = feedback.current_pose.pose.position.x
        cur_y = feedback.current_pose.pose.position.y
        dist = feedback.distance_remaining
        recovers = feedback.number_of_recoveries
        sec = feedback.navigation_time.sec
        trajectory.append((cur_x, cur_y))
        if abs(cur_y) > max_y_dev:
            max_y_dev = abs(cur_y)
        if i % 2 == 0:
            print(f\"[{sec:02d}s] Pose: ({cur_x:.3f}, {cur_y:.3f}) | Dist: {dist:.2f}m | Recoveries: {recovers}\")
    if (time.time() - start_time) > max_duration:
        print(f\"[TIMEOUT] Navigation exceeded {max_duration}s allowance without reaching goal. Cancelling task...\")
        navigator.cancelTask()
        while not navigator.isTaskComplete():
            time.sleep(0.1)
        print(\"----------------------------------------------------------\")
        print(\"RESULT: TIMEOUT - Goal is unreachable; task timed out and was aborted by Nav2.\")
        sys.exit(1)
    time.sleep(0.5)

result = navigator.getResult()
print(\"----------------------------------------------------------\")
if result == TaskResult.SUCCEEDED:
    print(f\"RESULT: SUCCESS - Goal reached successfully!\")
    if trajectory:
        fx, fy = trajectory[-1]
        print(f\"Final Pose: ({fx:.3f}, {fy:.3f}), Max lateral deviation: {max_y_dev:.3f}m\")
    sys.exit(0)
elif result == TaskResult.CANCELED:
    print(\"RESULT: CANCELED - Goal navigation was canceled.\")
    sys.exit(1)
elif result == TaskResult.FAILED:
    print(\"RESULT: FAILED - Nav2 could not find a path or aborted the goal.\")
    sys.exit(1)
else:
    print(f\"RESULT: UNKNOWN ({result})\")
    sys.exit(1)
'
"

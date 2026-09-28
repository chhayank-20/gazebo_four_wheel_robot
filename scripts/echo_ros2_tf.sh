#!/usr/bin/env bash
set -e

CONTAINER_NAME="ros2_gazebo_robot"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Start it first with: ./scripts/start_docker_sim.sh"
    exit 1
fi

echo "Echoing ROS 2 /tf (Press Ctrl+C to exit)..."
docker exec -it "${CONTAINER_NAME}" bash -c '
    source /opt/ros/jazzy/setup.bash
    ros2 topic echo /tf
'

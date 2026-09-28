#!/usr/bin/env bash
set -e

CONTAINER_NAME="ros2_gazebo_robot"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Start it first with: ./scripts/start_docker_sim.sh"
    exit 1
fi

echo "Launching RViz2 on virtual display :1 in container..."
docker exec -d "${CONTAINER_NAME}" bash -c '
    source /opt/ros/jazzy/setup.bash
    DISPLAY=:1 rviz2
'
echo "RViz2 opened! View it at http://localhost:8080/vnc.html"

#!/usr/bin/env bash
set -e

CONTAINER_NAME="ros2_gazebo_robot"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Start it first with: ./scripts/start_docker_sim.sh"
    exit 1
fi

echo "=========================================================="
echo " Echoing ROS 2 /scan (sensor_msgs/msg/LaserScan)"
echo " Press Ctrl+C to exit"
echo "=========================================================="

docker exec -it "${CONTAINER_NAME}" bash -c '
    source /opt/ros/jazzy/setup.bash
    ros2 topic echo /scan
'

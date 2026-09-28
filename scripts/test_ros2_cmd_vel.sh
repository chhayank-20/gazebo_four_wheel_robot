#!/usr/bin/env bash
set -e

CONTAINER_NAME="ros2_gazebo_robot"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Start it first with: ./scripts/start_docker_sim.sh"
    exit 1
fi

echo "=========================================================="
echo " Testing ROS 2 /cmd_vel -> ros_gz_bridge -> Gazebo Robot"
echo "=========================================================="

echo ""
echo "[1/3] Driving forward at 1.0 m/s for 3 seconds..."
docker exec "${CONTAINER_NAME}" bash -c '
    source /opt/ros/jazzy/setup.bash
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}" -r 10 &
    PUB_PID=$!
    sleep 3
    kill $PUB_PID 2>/dev/null || true
'

echo ""
echo "[2/3] Turning clockwise at -0.8 rad/s for 2 seconds..."
docker exec "${CONTAINER_NAME}" bash -c '
    source /opt/ros/jazzy/setup.bash
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.8}}" -r 10 &
    PUB_PID=$!
    sleep 2
    kill $PUB_PID 2>/dev/null || true
'

echo ""
echo "[3/3] Stopping robot..."
docker exec "${CONTAINER_NAME}" bash -c '
    source /opt/ros/jazzy/setup.bash
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1
'

echo ""
echo "=========================================================="
echo " Robot movement test completed!"
echo " Check latest position with: ./scripts/echo_ros2_odom.sh"
echo "=========================================================="

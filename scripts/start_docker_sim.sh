#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${PROJECT_DIR}"

echo "=========================================================="
echo " Building and Starting ROS 2 Jazzy + Gazebo Harmonic Stack"
echo "=========================================================="

docker compose up -d --build

echo ""
echo "Waiting for web visualizer to come online..."
for i in {1..30}; do
    if curl -s -o /dev/null -w "%{http_code}" http://localhost:8080 | grep -q "200\|302"; then
        echo "Web visualizer is online!"
        break
    fi
    sleep 1
done

VNC_URL="http://localhost:8080/vnc.html?autoconnect=true&resize=remote"
echo ""
echo "=========================================================="
echo " Simulation Stack is Running!"
echo ""
echo " 🌐 Open Simulation GUI in your browser:"
echo "    ${VNC_URL}"
echo ""
echo " Helpful commands:"
echo "   Drive Robot:  ./scripts/test_ros2_cmd_vel.sh"
echo "   Monitor Odom: ./scripts/echo_ros2_odom.sh"
echo "   Monitor TF:   ./scripts/echo_ros2_tf.sh"
echo "   Stop Stack:   ./scripts/stop_docker_sim.sh"
echo "=========================================================="

# Automatically open in default macOS browser if available
if command -v open >/dev/null 2>&1; then
    open "${VNC_URL}" || true
fi

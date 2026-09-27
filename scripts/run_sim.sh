#!/usr/bin/env bash
# Simulation launcher for the 4-wheel robot world in Gazebo Sim
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Ensure Gazebo loopback communication on macOS
export GZ_IP=127.0.0.1

cd "$PROJECT_DIR"

echo "=========================================================="
echo " Launching Gazebo Sim Four-Wheel Robot Simulation"
echo " World: worlds/robot_world.sdf"
echo " (Close window or press Ctrl+C to exit)"
echo "=========================================================="

gz sim -r worlds/robot_world.sdf "$@"

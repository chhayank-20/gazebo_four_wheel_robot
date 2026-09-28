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
echo " (Server running in background, GUI in foreground)"
echo " (Close GUI window or press Ctrl+C to exit)"
echo "=========================================================="

# Cleanup function to kill server when GUI exits
cleanup() {
    echo -e "\nShutting down Gazebo server..."
    if [ -n "$SERVER_PID" ]; then
        kill "$SERVER_PID" 2>/dev/null || true
        wait "$SERVER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# 1. Start Gazebo Server in the background (-s)
gz sim -s -r worlds/robot_world.sdf "$@" &
SERVER_PID=$!

# Give the server a moment to initialize physics and topics
sleep 2

# 2. Start Gazebo GUI in the foreground (-g)
gz sim -g

#!/usr/bin/env bash
set -e

# ==========================================================
#  Autonomous Office Demo Launcher
#  Demonstration Route:
#    Lobby -> Experience Centre (Living Demo) -> Board Room
#  Stack:
#    ROS 2 Jazzy + Gazebo Harmonic + Nav2 + noVNC Web Visualizer
# ==========================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${PROJECT_DIR}"

MODE="demo"
FORCE_RESTART=true

while [[ $# -gt 0 ]]; do
    case "$1" in
        --all|--full)
            MODE="all"
            shift
            ;;
        --no-restart)
            FORCE_RESTART=false
            shift
            ;;
        --help|-h)
            echo "Usage: ./scripts/run_autonomous_demo.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --all, --full    Run full 4-leg tour (Lobby -> Living Demo -> Board Room -> Accounts -> Lobby)"
            echo "  --no-restart     Skip restarting container if already running"
            echo "  -h, --help       Show this help message"
            echo ""
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage."
            exit 1
            ;;
    esac
done

echo "=========================================================="
echo " Casadigi Office Autonomous Navigation Demo"
echo " Route: Lobby -> Experience Centre (Living Demo) -> Board Room"
echo " Target Duration: ~1.5 - 2 minutes"
echo "=========================================================="

# 1. Check Docker
if ! docker info >/dev/null 2>&1; then
    echo "[ERROR] Docker is not running or accessible. Please start Docker and retry."
    exit 1
fi

# 2. Check or start container
CONTAINER_NAME="ros2_gazebo_robot"
CONTAINER_RUNNING=$(docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" -q)

if [ -z "${CONTAINER_RUNNING}" ]; then
    echo "[1/4] Starting containerized robotics stack with Docker Compose..."
    docker compose up -d
else
    if [ "${FORCE_RESTART}" = true ]; then
        echo "[1/4] Resetting simulation to initial Lobby spawn pose..."
        docker restart "${CONTAINER_NAME}" >/dev/null
    else
        echo "[1/4] Container is already running."
    fi
fi

# 3. Wait for Web Visualizer (noVNC on port 8080)
echo "[2/4] Verifying web visualizer (noVNC)..."
VNC_ONLINE=false
for i in {1..20}; do
    if curl -s -o /dev/null -w "%{http_code}" http://localhost:8080 | grep -q "200\|302"; then
        VNC_ONLINE=true
        break
    fi
    sleep 1
done

VNC_URL="http://localhost:8080/vnc.html?autoconnect=true&resize=remote"
if [ "${VNC_ONLINE}" = true ]; then
    echo "      Web GUI is active at: ${VNC_URL}"
else
    echo "      [WARN] Web GUI port 8080 not answering yet, proceeding..."
fi

# Open browser if on macOS
if command -v open >/dev/null 2>&1; then
    open "${VNC_URL}" 2>/dev/null || true
fi

# 4. Wait for Nav2 Stack to be Active
echo "[3/4] Checking readiness of ROS 2 Jazzy, Gazebo Harmonic, and Nav2 stack..."
NAV2_READY=false
for i in {1..30}; do
    if docker exec "${CONTAINER_NAME}" bash -c "cat /tmp/nav2.log 2>/dev/null | grep -q 'Managed nodes are active'"; then
        NAV2_READY=true
        break
    fi
    sleep 1
done

if [ "${NAV2_READY}" = false ]; then
    echo "[ERROR] Nav2 stack did not reach ACTIVE state within timeout. Check /tmp/nav2.log inside container."
    docker exec "${CONTAINER_NAME}" tail -n 30 /tmp/nav2.log 2>/dev/null || true
    exit 1
fi
echo "      Nav2 stack is ACTIVE and ready!"

# 5. Execute Autonomous Demo
echo "[4/4] Launching autonomous navigation demo (mode: ${MODE})..."
echo ""

docker exec -e PYTHONUNBUFFERED=1 "${CONTAINER_NAME}" bash -c "
source /opt/ros/jazzy/setup.bash
python3 /workspace/gazebo_four_wheel_robot/scripts/test_office_navigation.py --leg ${MODE}
"
DEMO_EXIT_CODE=$?

echo ""
if [ ${DEMO_EXIT_CODE} -eq 0 ]; then
    echo "=========================================================="
    echo " DEMO COMPLETED SUCCESSFULLY!"
    echo "=========================================================="
else
    echo "=========================================================="
    echo " [ERROR] DEMO ENCOUNTERED A FAILURE (exit code ${DEMO_EXIT_CODE})"
    echo "=========================================================="
fi

exit ${DEMO_EXIT_CODE}

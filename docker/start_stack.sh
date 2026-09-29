#!/usr/bin/env bash
set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="${PYTHONPATH}:/workspace/gazebo_four_wheel_robot"

echo "=========================================================="
echo " Starting Full Containerized Robotics Stack"
echo " OS: Ubuntu 24.04 ARM64"
echo " ROS 2: Jazzy Jalisco"
echo " Gazebo: Harmonic (gz-sim 8.15.0)"
echo " Bridge: ros_gz_bridge"
echo " Web Visualizer: noVNC (http://localhost:8080)"
echo "=========================================================="

export DISPLAY=:1
export RESOLUTION="${RESOLUTION:-1280x800x24}"

# 1. Start Virtual Framebuffer (Xvfb)
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
echo "[1/6] Starting virtual framebuffer Xvfb on display ${DISPLAY} (${RESOLUTION})..."
Xvfb ${DISPLAY} -screen 0 ${RESOLUTION} &
XVFB_PID=$!
sleep 1

# 2. Start Openbox Window Manager
echo "[2/6] Starting Openbox window manager..."
openbox &
OPENBOX_PID=$!
sleep 1

# 3. Start x11vnc
echo "[3/6] Starting x11vnc server on port 5900..."
x11vnc -display ${DISPLAY} -forever -shared -nopw -rfbport 5900 -bg
sleep 1

# 4. Start websockify / noVNC on port 8080
echo "[4/6] Starting noVNC web server on port 8080..."
websockify --web /usr/share/novnc 8080 localhost:5900 &
WEBSOCKIFY_PID=$!
sleep 1

# 5. Start Gazebo Sim Server (Headless physics)
echo "[5/6] Starting Gazebo Harmonic Server (worlds/robot_world.sdf)..."
gz sim -s -r worlds/robot_world.sdf &
GZ_SERVER_PID=$!
sleep 2

# 6. Start ros_gz_bridge
echo "[6/7] Starting ros_gz_bridge (/cmd_vel, /odom, /tf, /scan, /clock)..."
ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
    /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    /model/four_wheel_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    --ros-args -r /model/four_wheel_robot/tf:=/tf &
BRIDGE_PID=$!
sleep 2

# 7. Start static transform publishers (base_link -> laser_link, map -> odom)
echo "[7/7] Publishing static transforms (base_link -> laser_link, map -> odom)..."
ros2 run tf2_ros static_transform_publisher \
    --x 0.25 --y 0 --z 0.16 --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id laser_link &
STATIC_TF_PID=$!

ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id map --child-frame-id odom &
MAP_TF_PID=$!
sleep 1

# 8. Start Nav2 Navigation Stack
echo "Starting Nav2 Navigation Stack..."
ros2 launch /workspace/gazebo_four_wheel_robot/launch/nav2.launch.py > /tmp/nav2.log 2>&1 &
NAV2_PID=$!
sleep 2

# 9. Start Gazebo Sim GUI on the virtual display
echo "Launching Gazebo Sim GUI on virtual display..."
gz sim -g &
GZ_GUI_PID=$!

# Initial camera framing for clean view of robot and Lobby (Milestone 4A Task 3)
(
    sleep 4
    echo "Framing initial Gazebo camera (Lobby & corridor view)..."
    gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 \
        --req 'pose: {position: {x: -1.8, y: -6.0, z: 5.5}, orientation: {x: 0.2588, y: 0.0, z: 0.0, w: 0.9659}}' >/dev/null 2>&1 || true
) &

# 10. Start Robot Control FastAPI Server
echo "Starting Robot Control API server on port 8000..."
python3 -m uvicorn api.server:app --host 0.0.0.0 --port 8000 > /tmp/api_server.log 2>&1 &
API_PID=$!

echo "=========================================================="
echo " Stack is LIVE and ready!"
echo " Web GUI available at: http://localhost:8080/vnc.html?autoconnect=true&resize=remote"
echo " Robot Control API available at: http://localhost:8000"
echo "=========================================================="

# Cleanup handler
cleanup() {
    echo ""
    echo "Stopping all processes..."
    kill ${API_PID} 2>/dev/null || true
    kill ${GZ_GUI_PID} 2>/dev/null || true
    kill ${NAV2_PID} 2>/dev/null || true
    kill ${MAP_TF_PID} 2>/dev/null || true
    kill ${STATIC_TF_PID} 2>/dev/null || true
    kill ${BRIDGE_PID} 2>/dev/null || true
    kill ${GZ_SERVER_PID} 2>/dev/null || true
    kill ${WEBSOCKIFY_PID} 2>/dev/null || true
    pkill -f x11vnc 2>/dev/null || true
    kill ${OPENBOX_PID} 2>/dev/null || true
    kill ${XVFB_PID} 2>/dev/null || true
    echo "Stack stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Keep script running
wait

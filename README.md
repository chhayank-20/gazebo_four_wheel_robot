# Four-Wheel Mobile Robot Simulation & ROS 2 Jazzy Integration

A minimal, fully verified four-wheel mobile robot simulation in **Gazebo Sim** (Gazebo Harmonic / `gz-sim8`) integrated with **ROS 2 Jazzy Jalisco** and an interactive **Browser Web Visualizer (noVNC)**.

---

## 1. System Architecture

The complete robotics stack runs inside an Ubuntu 24.04 ARM64 container on macOS (via Docker / Colima), completely isolating dependencies while enabling full 3D GUI visualization through any macOS web browser.

```
┌────────────────────────────────────────────────────────────────────────┐
│ macOS Host (Apple Silicon)                                             │
│                                                                        │
│   Web Browser (Safari / Chrome / Firefox)                              │
│      └─▶ http://localhost:8080/vnc.html (3D Simulation View & Orbit)  │
│                                                                        │
│   Native macOS Gazebo Installation (Preserved & Untouched)             │
│      └─▶ ./scripts/run_sim.sh (Optional native macOS-only mode)        │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Port 8080 (HTTP + WebSockets)
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│ Container: ros2_gazebo_robot (Ubuntu 24.04 ARM64)                      │
│                                                                        │
│   ┌────────────────────────────────────────────────────────────────┐   │
│   │ ROS 2 Jazzy Jalisco                                            │   │
│   │   • /cmd_vel (geometry_msgs/msg/Twist)                         │   │
│   │   • /odom    (nav_msgs/msg/Odometry)                           │   │
│   │   • /tf      (tf2_msgs/msg/TFMessage)                          │   │
│   │   • /scan    (sensor_msgs/msg/LaserScan)                       │   │
│   └───────────────────────▲─────────────────▲──────────────────────┘   │
│                           │                 │                          │
│               ros_gz_bridge (parameter_bridge)                         │
│                           │                 │                          │
│   ┌───────────────────────▼─────────────────▼──────────────────────┐   │
│   │ Gazebo Harmonic (gz-sim 8.15.0)                                │   │
│   │   • Gazebo Server (Headless DART physics)                      │   │
│   │   • DiffDrive System Plugin (Skid-steer 4WD kinematics)        │   │
│   │   • Gazebo GUI Client (3D OGRE2 viewport)                      │   │
│   └────────────────────────────────┬───────────────────────────────┘   │
│                                    │ Display :1 (llvmpipe OpenGL 4.5)  │
│   ┌────────────────────────────────▼───────────────────────────────┐   │
│   │ Web Visualization Stack                                        │   │
│   │   • Xvfb (Virtual Framebuffer, 1280x800x24)                    │   │
│   │   • Openbox (Window Manager)                                   │   │
│   │   • x11vnc (VNC Server on port 5900)                           │   │
│   │   • Websockify + noVNC (HTML5 Canvas web client on port 8080) │   │
│   └────────────────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Directory Structure

```
gazebo_four_wheel_robot/
├── docker/
│   ├── Dockerfile            # Ubuntu 24.04 ARM64 + ROS 2 Jazzy + Gazebo Harmonic + noVNC
│   └── start_stack.sh        # Container startup script (Xvfb, openbox, vnc, gz-server, bridge, gz-gui)
├── docker-compose.yml        # Docker Compose service definition with port 8080 & volume mount
├── models/
│   └── four_wheel_robot/
│       ├── model.config      # Model metadata
│       └── model.sdf         # 4-wheel robot SDF definition (chassis, 4 wheels, DiffDrive plugin)
├── worlds/
│   └── robot_world.sdf       # Simulation world (ground plane, sun, walls, obstacles, robot)
├── scripts/
│   ├── start_docker_sim.sh   # One-click start: builds/boots container & opens browser visualizer
│   ├── stop_docker_sim.sh    # Stops container stack cleanly
│   ├── test_ros2_cmd_vel.sh  # Automated ROS 2 motion test (forward, turn, stop)
│   ├── echo_ros2_odom.sh     # Echoes ROS 2 /odom topic
│   ├── echo_ros2_tf.sh       # Echoes ROS 2 /tf topic
│   ├── echo_ros2_scan.sh     # Echoes ROS 2 /scan (LaserScan) topic
│   ├── open_rviz.sh          # Launches RViz2 in the virtual desktop
│   ├── run_sim.sh            # Native macOS Gazebo launcher (preserved)
│   └── test_movement.sh      # Native macOS Gazebo transport test (preserved)
└── README.md                 # Complete documentation
```

---

## 3. Quick Start (ROS 2 + Gazebo + Web GUI)

### Step 1: Launch the Stack

Run the launch script from the project root:

```bash
./scripts/start_docker_sim.sh
```

Or via standard Docker Compose:

```bash
docker compose up -d --build
```

### Step 2: Open the 3D Simulation View

Visit the following URL in Safari, Chrome, or Firefox on macOS:

```
http://localhost:8080/vnc.html?autoconnect=true&resize=remote
```

You will see the **Gazebo Sim GUI** rendered in real-time with:
- Full 3D camera controls: Left-click to orbit, Right-click to zoom, Middle-click to pan.
- The 4-wheel robot, arena walls, and obstacle blocks.
- Real-time simulation state and entity tree.

---

## 4. Controlling & Verifying via ROS 2

### Automated Motion Test

Run the pre-configured movement test script:

```bash
./scripts/test_ros2_cmd_vel.sh
```

This commands:
1. Drive forward ($1.0\,\text{m/s}$ for 3s).
2. Turn clockwise ($-0.8\,\text{rad/s}$ for 2s).
3. Stop ($0.0\,\text{m/s}$).

### Publishing Custom Velocity Commands

Publish directly from your macOS terminal into the container:

```bash
# Drive Forward (1.0 m/s)
docker exec ros2_gazebo_robot bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.0}}' -1"

# Turn Left (1.0 rad/s)
docker exec ros2_gazebo_robot bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 1.0}}' -1"

# Stop
docker exec ros2_gazebo_robot bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' -1"
```

### Monitoring 2D LiDAR Scan (`/scan`)

Inspect the streaming 2D laser scan:

```bash
./scripts/echo_ros2_scan.sh
```

Sample output:
```yaml
header:
  stamp:
    sec: 41
    nanosec: 200000000
  frame_id: laser_link
angle_min: -3.1415927
angle_max: 3.1415927
angle_increment: 0.0175019
range_min: 0.1
range_max: 10.0
ranges: [ ... ]
```

### Monitoring Odometry (`/odom`)

Inspect the streaming odometry calculated by Gazebo and bridged into ROS 2:

```bash
./scripts/echo_ros2_odom.sh
```

Sample output:
```yaml
header:
  stamp:
    sec: 42
    nanosec: 150000000
  frame_id: odom
child_frame_id: chassis
pose:
  pose:
    position:
      x: 2.8541
      y: 0.0001
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.982
      w: -0.188
twist:
  twist:
    linear: {x: 0.0, y: 0.0, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}
```

### Monitoring TF Frames (`/tf`)

Inspect the live transform `odom -> chassis`:

```bash
./scripts/echo_ros2_tf.sh
```

### Optional: Launch RViz2

To visualize ROS 2 odometry and TF frames in RViz2 alongside Gazebo Sim:

```bash
./scripts/open_rviz.sh
```

RViz2 will appear inside the web browser desktop on `http://localhost:8080/vnc.html`.

### Stopping the Stack

```bash
./scripts/stop_docker_sim.sh
```

---

## 5. Native macOS Gazebo Simulation (Preserved)

Your native macOS Gazebo Harmonic installation remains completely untouched and functional:

```bash
# Launch native macOS Gazebo GUI & Server
./scripts/run_sim.sh

# Run native Gazebo transport movement test
./scripts/test_movement.sh
```

---

## 6. Technical Design Decisions & Architecture Insights

1. **Why Full Containerization with noVNC?**
   - **ZeroMQ Dynamic Ports**: Gazebo Transport binds ephemeral ZeroMQ TCP ports for each topic and service and relies on UDP multicast discovery. Because macOS container networking (Colima/Docker with `gvproxy` / SLIRP NAT) cannot dynamically forward unpredictable ephemeral ports or bridge multicast across macOS and Linux, running Gazebo Server and GUI in the same container guarantees zero network packet loss and 60 FPS performance.
   - **Host Cleanliness**: macOS host requires zero X11/XQuartz installations, zero security token fiddling, and zero firewall modifications.
   - **Hardware-Free Software Rendering**: The container leverages Mesa `llvmpipe` providing **OpenGL 4.5**, enabling Gazebo's OGRE2 rendering engine to run smoothly without requiring host GPU passthrough.
   - **Browser-Accessible**: Any standard browser (Safari, Chrome) connects directly to `http://localhost:8080` via HTML5 WebSockets.

2. **Topic Bridging (`ros_gz_bridge`)**:
   - `parameter_bridge` bridges bidirectional velocity `/cmd_vel` (`geometry_msgs/msg/Twist` $\leftrightarrow$ `gz.msgs.Twist`).
   - Streams Gazebo `/odom` (`gz.msgs.Odometry`) to ROS 2 `/odom` (`nav_msgs/msg/Odometry`).
   - Bridges model pose `/model/four_wheel_robot/tf` (`gz.msgs.Pose_V`) remapped to standard ROS 2 `/tf` (`tf2_msgs/msg/TFMessage`).

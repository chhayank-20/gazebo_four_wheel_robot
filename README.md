# Four-Wheel Mobile Robot Gazebo Simulation (No ROS 2)

A minimal, fully autonomous four-wheel mobile robot simulation in **Gazebo Sim** (Gazebo Harmonic / `gz-sim8`) without any ROS 2 dependencies.

---

## 1. Environment & Version

- **Gazebo Suite**: Gazebo Harmonic
- **Simulator Version**: `gz-sim8` (v8.15.0)
- **Tools**: `gz-tools2`, `gz-msgs10`, `gz-transport13`, `sdformat14`
- **Physics Engine**: DART (`gz-physics7-dartsim-plugin`)
- **Host OS**: macOS (Apple Silicon / Darwin)

---

## 2. Project Structure

```
gazebo_four_wheel_robot/
├── models/
│   └── four_wheel_robot/
│       ├── model.config      # Model metadata and description
│       └── model.sdf         # 4-wheel robot SDF definition (chassis, 4 wheels, DiffDrive plugin)
├── worlds/
│   └── robot_world.sdf       # Simulation world (ground plane, sun, walls, obstacles, robot)
├── plugins/
│   └── README.md             # Overview of native Gazebo Sim plugins utilized
├── scripts/
│   ├── run_sim.sh            # One-click launcher for the simulation
│   └── test_movement.sh      # Automated CLI test demonstrating all 5 movement states
└── README.md                 # Documentation and test report
```

---

## 3. Robot Design & Kinematics

The robot is a skid-steer 4-wheel drive mobile robot built strictly with basic geometric primitives:

- **Chassis**:
  - Shape: Rectangular box ($0.8\,\text{m} \times 0.5\,\text{m} \times 0.15\,\text{m}$)
  - Mass: $12.0\,\text{kg}$
  - Color: Vivid Blue
  - Clearance: $12.5\,\text{cm}$ above ground
- **Wheels (4 Cylinders)**:
  - Shape: Cylinders ($R = 0.15\,\text{m}$, Width $= 0.08\,\text{m}$)
  - Placement:
    - Front-Left: $(+0.25, +0.30, 0.0)$ relative to chassis
    - Rear-Left: $(-0.25, +0.30, 0.0)$ relative to chassis
    - Front-Right: $(+0.25, -0.30, 0.0)$ relative to chassis
    - Rear-Right: $(-0.25, -0.30, 0.0)$ relative to chassis
  - Wheelbase: $0.50\,\text{m}$ | Track Width: $0.60\,\text{m}$
  - Mass: $2.0\,\text{kg}$ each
  - Joints: 4 revolute joints with rotation axis along local $\mathbf{Y}$ ($[0, 1, 0]$)
  - Surface Friction: Directional ODE friction ($\mu_1 = 1.0$ forward traction along $\mathbf{X}$, $\mu_2 = 0.5$, $\text{slip}_2 = 0.05$ lateral compliance for smooth skid-steering)

---

## 4. How the Robot is Controlled (Gazebo-Native)

Movement control is provided by the native Gazebo Sim plugin:
`gz-sim-diff-drive-system` (`gz::sim::systems::DiffDrive`).

- **Control Input Topic**: `/cmd_vel` (`gz.msgs.Twist`)
- **Odometry Feedback Topic**: `/odom` (`gz.msgs.Odometry`)
- **Transform Broadcast**: `/model/four_wheel_robot/tf` (`gz.msgs.Pose_V`)

Both left wheels (`front_left_wheel_joint`, `rear_left_wheel_joint`) and both right wheels (`front_right_wheel_joint`, `rear_right_wheel_joint`) are controlled in coordinated skid-steering mode.

---

## 5. Launch Commands

### Launching with Gazebo GUI (Visual Mode)

From the `gazebo_four_wheel_robot` directory:

```bash
export GZ_IP=127.0.0.1
gz sim -r worlds/robot_world.sdf
```

Or using the helper script:
```bash
./scripts/run_sim.sh
```

### Launching in Headless Server Mode (CLI / Background)

```bash
export GZ_IP=127.0.0.1
gz sim -s -r worlds/robot_world.sdf
```

---

## 6. Testing Movement via CLI

All commands use pure Gazebo transport (`gz topic`):

### 1. Move Forward (1.0 m/s)
```bash
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: 0.0}"
```

### 2. Move Backward (-1.0 m/s)
```bash
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: -1.0}, angular: {z: 0.0}"
```

### 3. Turn Left (1.0 rad/s counter-clockwise)
```bash
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 1.0}"
```

### 4. Turn Right (-1.0 rad/s clockwise)
```bash
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: -1.0}"
```

### 5. Stop (0.0 m/s)
```bash
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"
```

### Inspecting Odometry in Real Time
```bash
gz topic -e -t /odom -n 1
```

Or execute the complete test suite automatically:
```bash
./scripts/test_movement.sh
```

---

## 7. Validation Results

| Test Case | Commanded Input | Observed Twist / Pose Change | Result |
| :--- | :--- | :--- | :--- |
| **Spawn Stability** | Idle | $z = 0.100\,\text{m}$ (wheels touch $z=0.00$), tilt angles $\approx 0^\circ$ | **PASS** |
| **Forward** | `linear.x = 1.0` | $x$ progressed $0.0 \to +2.08\,\text{m}$, linear velocity $= 1.0\,\text{m/s}$ | **PASS** |
| **Stop** | `linear.x = 0.0` | Velocity returned to $0.0\,\text{m/s}$, stopped cleanly | **PASS** |
| **Backward** | `linear.x = -1.0` | $x$ progressed $+6.44 \to +4.38\,\text{m}$, linear velocity $= -1.0\,\text{m/s}$ | **PASS** |
| **Turn Left** | `angular.z = +1.0` | Angular velocity $= +1.0\,\text{rad/s}$, heading turned CCW | **PASS** |
| **Turn Right** | `angular.z = -1.0` | Angular velocity $= -1.0\,\text{rad/s}$, heading turned CW | **PASS** |
| **Post-run Stability** | Repeated motions | Height stayed at $z = 0.10\,\text{m}$, roll/pitch $< 10^{-10}\,\text{rad}$ | **PASS** |

---

## 8. Problems Encountered & Resolutions

1. **Multicast UDP Routing on macOS**:
   - *Problem*: On macOS, when running `gz topic` or `gz sim`, Gazebo Transport's discovery mechanism raised `Exception sending a multicast message: No route to host`.
   - *Resolution*: Set `GZ_IP=127.0.0.1`. This forces Gazebo Transport to bind to the local loopback interface (`lo0`), resolving all discovery calls cleanly without network errors.

2. **Model Path Resolution without ROS**:
   - *Problem*: Standard relative path `<uri>../models/four_wheel_robot</uri>` inside the world SDF is rejected by SDFormat's parser.
   - *Resolution*: Configured `<uri>package://models/four_wheel_robot</uri>` along with a compliant `model.config`. SDFormat 1.8 automatically resolves `package://` relative to the current workspace root, allowing `gz sim worlds/robot_world.sdf` to load without complex environment setups.

3. **Skid-Steer Lateral Resistance**:
   - *Problem*: Four rigid wheels without lateral compliance can induce excessive friction and chattering during zero-radius yaw rotations.
   - *Resolution*: Tuned the wheel surface contact parameters with anisotropic friction ($\mu_1 = 1.0$ along the rolling axis, $\mu_2 = 0.5$ laterally with `slip2 = 0.05`), providing crisp forward traction while enabling smooth rotational sliding.

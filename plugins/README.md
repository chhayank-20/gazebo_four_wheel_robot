# Gazebo Simulation Plugins

This simulation relies purely on native Gazebo Sim (Harmonic / gz-sim 8) system plugins:

1. **`gz-sim-diff-drive-system` (`gz::sim::systems::DiffDrive`)**:
   - Implements skid-steer differential drive control across all four wheels (`front_left_wheel_joint`, `rear_left_wheel_joint`, `front_right_wheel_joint`, `rear_right_wheel_joint`).
   - Subscribes to `/cmd_vel` (`gz.msgs.Twist`).
   - Publishes odometry to `/odom` (`gz.msgs.Odometry`) and transforms to `/tf` (`gz.msgs.Pose_V`).

2. **`gz-sim-physics-system` (`gz::sim::systems::Physics`)**:
   - Manages DART physics simulation at 1ms timestep.

3. **`gz-sim-scene-broadcaster-system` (`gz::sim::systems::SceneBroadcaster`)**:
   - Broadcasts world model and visual state updates to GUI and clients.

4. **`gz-sim-user-commands-system` (`gz::sim::systems::UserCommands`)**:
   - Provides user and programmatic interaction services (pause, play, entity management).

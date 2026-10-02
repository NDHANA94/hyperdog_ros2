# hyperdog_bringup

Real robot launch files.

- `launch/robot.launch.xml`: robot_state_publisher, `ros2_control_node` with the
  `MitCanSystem` hardware interface, BLDC controller, locomotion controller, gamepad teleop.
- `launch/display.launch.xml`: publishes the robot model only.
- `config/robot_overrides.yaml`: parameters that differ from simulation (no motor model,
  Mahony attitude filter, torque based contact detection, no auto start).

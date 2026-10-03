# hyperdog_bldc_control

BLDC actuator layer of HyperDog.

| file | content |
|---|---|
| `include/.../bldc_motor_model.hpp` | header-only electro-mechanical motor model (current / back-EMF / thermal limits, current loop, friction) |
| `include/.../bldc_impedance_controller.hpp`, `src/bldc_impedance_controller.cpp` | `hyperdog_bldc_control/BldcImpedanceController` ros2_control controller |
| `include/.../mit_can_protocol.hpp` | MIT CAN protocol packing / unpacking |
| `src/mit_can_system.cpp` | `hyperdog_bldc_control/MitCanSystem` SocketCAN hardware interface (real robot) |
| `src/tools/mit_can_probe.cpp` | bench tool without ROS: scan the bus, stream a motor's feedback, set the zero, low-gain hold |
| `src/tools/joint_test.cpp` | ROS node: sine tracking or torque step on one joint through this controller, prints tracking error, peak current, saturation, temperature |
| `test/test_bldc_motor_model.cpp` | motor model and protocol unit tests |

The controller applies `tau = kp (q* - q) + kd (dq* - dq) + tau_ff` per joint at the
controller manager rate, passes it through the motor model of the joint's motor type and
writes the delivered torque to the `effort` command interface. Without new commands for
`command_timeout` seconds it falls back to damping. Motor types and the joint-to-motor
mapping are configured in `hyperdog_description/config/bldc_motors.yaml`; controller
options in `hyperdog_description/config/controllers.yaml`.

Topics: `~/commands` (`hyperdog_msgs/MotorCommands`, in), `~/motor_states`
(`hyperdog_msgs/MotorStates`: torque, current, power, winding temperature, saturation).
The controller parameter `command_latency` [s] delays the commands (robustness testing).

See [docs/hardware_bringup.md](../docs/hardware_bringup.md) for how to use the tools.

# hyperdog_description

Robot model and actuator configuration.

| file | content |
|---|---|
| `urdf/hyperdog.urdf.xacro` | top level model (args: `sim`, `use_lidar`, `use_camera`, `motors_file`, `controllers_file`, `can_interface`) |
| `urdf/common.xacro` | geometry, masses, joint limits; joint effort / velocity limits derived from the motor file |
| `urdf/leg.xacro` | leg macro (hip, thigh, shank, foot) |
| `urdf/sensors.xacro` | IMU, foot contact sensors, 3D lidar, RGB-D camera, ground truth odometry |
| `urdf/ros2_control.xacro` | `gz_ros2_control` (sim) or `MitCanSystem` (robot), CAN ids / directions |
| `config/bldc_motors.yaml` | **BLDC motor types and joint -> motor mapping** |
| `config/controllers.yaml` | controller manager, BLDC controller options |
| `meshes/` | visual meshes |

Geometry values must stay consistent with `hyperdog_locomotion/config/locomotion.yaml` (`robot.*`).

# HyperDog software architecture

## Layers and rates
| layer | package | rate | runs in |
|---|---|---|---|
| teleop / navigation | `hyperdog_teleop`, Nav2 (`hyperdog_navigation`) or any `/cmd_vel` source | event driven / 20 Hz | ROS nodes |
| terrain perception | `hyperdog_perception` (`height_map_node`) | camera rate, map 10 Hz | ROS node |
| locomotion controller | `hyperdog_locomotion` | 500 Hz (MPC re-solve 100 Hz) | ROS node, sim time in Gazebo |
| actuator impedance loop + BLDC model | `hyperdog_bldc_control` | 1 kHz | ros2_control controller manager |
| physics / hardware | Gazebo Harmonic (DART 1 kHz) or `MitCanSystem` | 1 kHz | Gazebo / CAN |

## Data flow
```
/cmd_vel, /hyperdog/command        depth camera / lidar -> height_map_node
        |                                     | /hyperdog/height_map (odom frame)
        v                                     v
 locomotion_node ---------------------------------------------+
   SensorData (joint_states, imu, foot contacts)              |
   LocomotionController::step()                               |  /odom, TF, /hyperdog/state
     estimation:  MahonyFilter | ContactEstimator |           |
                  KinematicKalmanFilter | GroundPlaneEstimator|
     planning:    DisturbanceMonitor -> GaitScheduler ->      |
                  body reference -> plan_foothold             |
                  (-> TerrainMap: edge avoidance, heights)    |
     control:     ConvexMPC / QPBalanceController -> forces   |
                  LegController -> MotorCommand               |
        |  /bldc_controller/commands (MotorCommands)          |
        v                                                     |
 BldcImpedanceController (ros2_control)                       |
   tau = kp (q* - q) + kd (dq* - dq) + tau_ff                 |
   BldcMotorModel -> effort command interface                 |
        |                                                     |
        v                                                     |
 gz_ros2_control (sim) | MitCanSystem (robot) -> joint_states-+
```

## State machine
```
PASSIVE --(LOCOMOTION / STAND requested)--> STAND_UP --(done)--> BALANCE <--> LOCOMOTION
   ^                                                                |   SIT requested
   +---------------- SIT (lower the body) <-------------------------+
   +---------------- fall detected (|roll| or |pitch| > fall_angle) from BALANCE / LOCOMOTION
                     (self_righting.enabled: false)

fall detected --> SELF_RIGHT --(upright)--> STAND_UP --> commanded mode
                      |  (max_attempts failed, or PASSIVE / SIT requested)
                      +--> PASSIVE
```
SELF_RIGHT waits in damping until the body is at rest and then decides from its orientation:
lying on a side, the lower legs swing over the body and roll it onto its belly; lying on the
back, a fast kick of the lower legs tips it onto a side first.
In BALANCE (and LOCOMOTION without a velocity command) the robot stands on four legs
using the QP balance controller. When the `DisturbanceMonitor` sees the capture point
leave the support polygon, or the body moves or tilts too much, the controller steps
with the recovery gait until the robot has settled.

## Conventions
- Legs: FR, FL, BR, BL. Joints per leg: hip (ab/ad), uleg (thigh), lleg (knee).
- Frames: `base_link` body frame (x forward, z up). The world / `odom` frame is z up.
  Ground reaction forces act on the robot and are expressed in the world frame.
- Units: SI.

## Configuration files
| file | consumer |
|---|---|
| `hyperdog_description/config/bldc_motors.yaml` | URDF limits + `BldcImpedanceController` motor models |
| `hyperdog_description/config/controllers.yaml` | controller manager, BLDC controller options |
| `hyperdog_locomotion/config/locomotion.yaml` | `locomotion_node` (all controller parameters) |
| `hyperdog_gazebo/config/bridge*.yaml` | ros_gz bridges |
| `hyperdog_teleop/config/joy_xbox.yaml` | gamepad mapping |
| `hyperdog_bringup/config/robot_overrides.yaml` | real robot overrides |

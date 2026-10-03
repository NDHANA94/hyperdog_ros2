# hyperdog_locomotion

Closed-loop locomotion controller of HyperDog (C++17, Eigen). The core library
(`hyperdog_locomotion_core`) has no ROS dependency; `locomotion_node` wraps it.

## Layout
```
include/hyperdog_locomotion/
  common/math.hpp                    types (Vec3, Mat43, ...) and rotation helpers
  kinematics/leg_kinematics.hpp      FK / IK / Jacobian, consistent with the URDF
  estimation/
    attitude_filter.hpp              Mahony filter (IMUs without orientation output)
    contact_estimator.hpp            schedule + sensor / torque contact fusion
    kinematic_kalman_filter.hpp      base position / velocity + foot positions
    ground_plane_estimator.hpp       terrain plane from the stance feet
  planning/
    gait_scheduler.hpp               phase based gaits, safe transitions
    foothold_planner.hpp             Raibert + capture point foot placement
    swing_trajectory.hpp             min-jerk swing trajectories
    disturbance_monitor.hpp          push detection -> automatic stepping
  control/
    qp_solver.hpp                    dense ADMM QP solver
    force_control_common.hpp         body state, friction pyramids
    qp_balance_controller.hpp        PD wrench + QP force distribution (standing)
    convex_mpc.hpp                   single rigid body convex MPC (stepping)
    leg_controller.hpp               stance / swing -> MIT-mode joint commands
    self_righting.hpp                getting back up after a fall
  controller_types.hpp               SensorData, Command, MotorCommand, Diagnostics
  controller_config.hpp              ControllerConfig (mirrors config/locomotion.yaml)
  locomotion_controller.hpp          state machine + per tick pipeline
src/<same layout>.cpp
src/ros/locomotion_node.cpp          ROS 2 node (topics, timers, TF)
src/ros/parameter_loader.cpp         ROS parameters -> ControllerConfig
src/ros/markers.cpp                  RViz markers (feet, targets, support polygon, forces)
test/test_{kinematics,estimation,planning,control}.cpp
config/locomotion.yaml               every tunable parameter
```

## Pipeline (500 Hz)
attitude -> contact estimation -> Kalman filter -> gait selection / disturbance
monitor -> gait scheduler -> body reference -> footholds -> ground reaction forces
(MPC re-solved at 100 Hz while stepping, QP while standing) -> leg controller.

## Interfaces
| direction | topic | type |
|---|---|---|
| in | `joint_states` | `sensor_msgs/JointState` |
| in | `imu` | `sensor_msgs/Imu` |
| in | `hyperdog/foot_contact/{FR,FL,BR,BL}` | `ros_gz_interfaces/Contacts` |
| in | `cmd_vel` | `geometry_msgs/Twist` |
| in | `hyperdog/command` | `hyperdog_msgs/LocomotionCommand` |
| out | `bldc_controller/commands` | `hyperdog_msgs/MotorCommands` |
| out | `hyperdog/state` | `hyperdog_msgs/LocomotionState` |
| out | `odom`, TF `odom -> base_link` | `nav_msgs/Odometry` |
| out | `hyperdog/markers` (`publish_markers`) | `visualization_msgs/MarkerArray` |

Set `debug_log_file` to write a per-tick CSV (feet, targets, forces, state) for tuning.

# Contributing to hyperdog_ros2

## Repository layout
```
hyperdog_msgs/          message definitions
hyperdog_description/   URDF / xacro, BLDC motor parameters, ros2_control configuration
hyperdog_bldc_control/  BLDC impedance controller, motor model, CAN hardware interface
hyperdog_locomotion/    locomotion controller (ROS independent core + ROS node)
hyperdog_gazebo/        simulation worlds, bridges, launch files, validation scenarios
hyperdog_teleop/        gamepad teleoperation
hyperdog_bringup/       real robot launch files
docs/                   architecture notes and validation reports
```
Dependencies only point "down" this list: messages <- description / actuators <-
locomotion <- simulation / teleop / bringup. The locomotion core library must stay free
of ROS dependencies; ROS code lives in `hyperdog_locomotion/src/ros/`.

## Conventions
- **Language:** C++17 for all code. Configuration in YAML parameter files, launch files in XML.
- **Style:** ROS 2 C++ style, enforced by `ament_uncrustify` and `ament_cpplint`
  (100 columns). Reformat with `ament_uncrustify --reformat <package>`.
- **Files:** one class per header / source pair, placed in the subsystem directory it
  belongs to (`common`, `kinematics`, `estimation`, `planning`, `control` in
  `hyperdog_locomotion`). Header guards follow the path
  (`HYPERDOG_LOCOMOTION__CONTROL__CONVEX_MPC_HPP_`).
- **Parameters:** no hard coded tuning values. Every tunable value is a field of a
  parameter struct with a default, exposed in the package's YAML file and loaded by
  the node. Physical constants that are not meant to be tuned are named `constexpr`s.
- **Units:** SI units everywhere (m, rad, s, N, Nm, A, V). Document units in comments
  and YAML.
- **Frames and order:** legs are ordered FR, FL, BR, BL and joints hip, uleg, lleg. Forces
  computed by the balance controllers act on the robot and are expressed in the world frame.
- **License header:** every C++ file starts with the Apache-2.0 header
  (`// Copyright 2024 W.M. Nipun Dhananjaya Weerakkodi` followed by the standard
  "Licensed under the Apache License, Version 2.0" notice; copy it from any existing file).
  It is checked by `ament_copyright`.

## Before opening a pull request
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --verbose          # unit tests + linters
ros2 launch hyperdog_gazebo validate.launch.xml scenario:=full
ros2 launch hyperdog_gazebo validate.launch.xml scenario:=terrain world:=terrain
```
Changes that affect the robot's behaviour must keep the validation scenarios passing.
Update the reports in `docs/validation/` when the metrics change noticeably.

# hyperdog_ros2 (branch `latest`)

HyperDog is an open-source quadruped robot built on ROS 2. This branch upgrades the
project to **ROS 2 Jazzy + Gazebo Harmonic** and replaces the hobby servos with
**BLDC actuators** driven in impedance ("MIT") mode. It adds a **closed-loop locomotion
controller written in C++**: state estimation, convex MPC / QP force control,
capture-point stepping and automatic push recovery. Everything is validated in
simulation with automated scenarios. The original ROS 2 Foxy / servo implementation
is still available on the [`foxy`](https://github.com/NDHANA94/hyperdog_ros2/tree/foxy) branch.

### Publications
- [`HyperDog` | IEEE SMC-2022](https://ieeexplore.ieee.org/document/9945526)
- [`DogTouch` | IEEE VTC-2022](https://ieeexplore.ieee.org/document/9860815)
- [`HyperGuider` | IEEE SMC-2022](https://ieeexplore.ieee.org/document/9945364)
- [`HyperPalm` | IEEE SMC-2022](https://ieeexplore.ieee.org/document/9945223)

### YouTube
- [HyperDog Demo](https://youtu.be/Dx1U2J1avO0)
- [IEEE SMC-2023 presentation](https://youtu.be/mFgaS3f5-pw)
- [HyperDog Playlist](https://www.youtube.com/watch?v=1CIkmu7lIlY&list=PL8ZSjYfd0W-1BoGKr-xrBt6LwaWn_EVaN)

---

## Architecture

```
 /cmd_vel, gamepad --> hyperdog_locomotion (500 Hz): state estimation, gait + footholds,
                       convex MPC / QP force control, push recovery
                          | MotorCommands (q, dq, kp, kd, tau_ff)
                          v
                       hyperdog_bldc_control (1 kHz): impedance law + BLDC motor model
                          | joint effort
                          v
                       Gazebo Harmonic (sim)  |  MIT CAN BLDC drivers (robot)
```
See [docs/architecture.md](docs/architecture.md) for the data flow, state machine and conventions.
All code in this branch is C++; configuration is done with YAML parameter files and XML launch files.

## Packages

| package | content |
|---|---|
| `hyperdog_msgs` | `MotorCommands`, `MotorStates`, `LocomotionCommand`, `LocomotionState` (+ legacy Foxy messages) |
| `hyperdog_description` | xacro model, **`config/bldc_motors.yaml`** (actuator parameters), `config/controllers.yaml`, sensors |
| `hyperdog_bldc_control` | `BldcImpedanceController` (ros2_control controller + BLDC motor model), `MitCanSystem` hardware interface, bring-up tools `mit_can_probe` and `joint_test`, unit tests |
| `hyperdog_locomotion` | locomotion library (`*_core`), `locomotion_node`, **`config/locomotion.yaml`**, unit tests |
| `hyperdog_gazebo` | Gazebo Harmonic worlds (`flat`, `terrain`, `rough`, `stairs`, `slippery`), ros_gz bridges, `sim.launch.xml`, `validate.launch.xml`, `scenario_runner`, `robustness_campaign` |
| `hyperdog_teleop` | gamepad teleop (`/joy` -> `/cmd_vel` + `/hyperdog/command`), mapping in `config/joy_xbox.yaml` |
| `hyperdog_bringup` | real-robot launch (`robot.launch.xml`), robot parameter overrides |
| `hyperdog_perception` | terrain elevation map from the depth camera / lidar (`height_map_node`) for foothold selection |
| `hyperdog_navigation` | Nav2 configuration (lidar costmaps, omnidirectional MPPI), simulation launch, goal test |

Each package has its own README describing its files.

### Sensors (simulation)
| sensor | topic | notes |
|---|---|---|
| IMU (500 Hz, noisy) | `/hyperdog/imu` | orientation, gyro, accelerometer |
| 4 foot contact sensors | `/hyperdog/foot_contact/{FR,FL,BR,BL}` | `ros_gz_interfaces/Contacts` |
| 3D lidar, 16 beams, 360 deg | `/hyperdog/lidar/points` | `use_lidar:=true` (needs rendering) |
| RGB-D camera | `/hyperdog/camera/{image,depth_image,points,camera_info}` | `use_camera:=true` (needs rendering) |
| joint encoders / torques | `/joint_states`, `/bldc_controller/motor_states` | motor current, power, winding temperature, saturation |
| state estimate | `/odom`, TF `odom -> base_link`, `/hyperdog/state` | from the kinematic Kalman filter |
| controller visualisation | `/hyperdog/markers` | feet (red: contact), foot targets, support polygon, ground reaction forces (RViz `MarkerArray`) |
| ground truth | `/hyperdog/ground_truth` | simulation only (validation) |

## Requirements
- Ubuntu 24.04, [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html), Gazebo Harmonic (`ros-jazzy-ros-gz`)
- `sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro ros-jazzy-joy libeigen3-dev`

## Build
```bash
mkdir -p ~/hyperdog_ws/src && cd ~/hyperdog_ws/src
git clone -b latest https://github.com/NDHANA94/hyperdog_ros2.git
cd ~/hyperdog_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select hyperdog_bldc_control hyperdog_locomotion && colcon test-result
```

## Run the simulation
```bash
ros2 launch hyperdog_gazebo sim.launch.xml                       # flat world, GUI, lidar + camera
ros2 launch hyperdog_gazebo sim.launch.xml world:=terrain         # ramps and a plateau
ros2 launch hyperdog_gazebo sim.launch.xml world:=rough           # 1-3 cm random slabs (also: stairs, slippery)
ros2 launch hyperdog_gazebo sim.launch.xml headless:=true use_lidar:=false use_camera:=false
```
The robot spawns crouched, stands up automatically and waits for velocity commands.
```bash
# keyboard / any Twist source (body frame)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"
# gamepad
ros2 launch hyperdog_teleop teleop_joy.launch.xml use_sim_time:=true
# mode / gait / body pose (MODE_PASSIVE=0, MODE_STAND=1, MODE_LOCOMOTION=2, MODE_SIT=3)
ros2 topic pub --once /hyperdog/command hyperdog_msgs/msg/LocomotionCommand "{mode: 2, gait: walk, body_height: 0.22}"
# push the robot yourself (persistent wrench, then clear)
ros2 topic pub --once /world/hyperdog/wrench/persistent ros_gz_interfaces/msg/EntityWrench \
  "{entity: {name: 'hyperdog::base_link', type: 3}, wrench: {force: {y: 40.0}}}"
ros2 topic pub --once /world/hyperdog/wrench/clear ros_gz_interfaces/msg/Entity "{name: 'hyperdog::base_link', type: 3}"
```

Gamepad (Xbox layout): `START` stand/sit, `BACK` passive (e-stop), `A/B/X/Y` trot/walk/pace/bound,
left stick for translation, right stick for yaw, `LB` + left stick for body roll/pitch, d-pad for body height
(`LB` + d-pad for step height).

## BLDC actuators: adding your own motor
Edit [`hyperdog_description/config/bldc_motors.yaml`](hyperdog_description/config/bldc_motors.yaml)
(or pass your own file with `motors_file:=...`):

```yaml
bldc_controller:
  ros__parameters:
    joint_motor_types: ["my_motor"]          # one entry for all joints, or 12 entries (joint order)
    motors:
      my_motor:
        torque_constant: 0.10                # Kt [Nm/A] rotor side (<= 0 -> derived from kv_rpm_per_volt)
        kv_rpm_per_volt: 100.0
        phase_resistance: 0.2                # [Ohm]
        bus_voltage: 48.0                    # [V]
        max_current: 30.0                    # [A] peak
        rated_current: 10.0                  # [A] continuous
        gear_ratio: 9.0
        gearbox_efficiency: 0.9
        current_loop_bandwidth: 1000.0       # [Hz]
        coulomb_friction: 0.05               # [Nm]
        viscous_friction: 0.01               # [Nm s/rad]
        thermal_resistance: 2.5              # [K/W]
        thermal_capacitance: 50.0            # [J/K]
        derate_start_temperature: 90.0       # [degC]
        max_winding_temperature: 120.0       # [degC]
        # ... see the file for every parameter
```
The same file sets the URDF joint effort/velocity limits (peak torque, no-load speed) and
configures the per-joint motor model in the controller. The default actuator is a generic 48 V,
10:1 quasi-direct-drive motor (AK70-10 class): **27.7 Nm peak, 8.9 Nm continuous, 22.5 rad/s**.

## Locomotion parameters
Everything is in [`hyperdog_locomotion/config/locomotion.yaml`](hyperdog_locomotion/config/locomotion.yaml), including:
- `balance.controller`: `mpc` (convex MPC while stepping, QP while standing) or `qp`
- gaits (period, duty factor, phase offsets), body height, step height, velocity and acceleration limits
- `disturbance_recovery.*`: automatic stepping when the capture point leaves the support polygon
- `estimation.attitude_source` (`imu_orientation` | `mahony`), `estimation.contact_source` (`sensor` | `torque` | `schedule`)
- joint gains for stand-up, stance and swing, swing cartesian impedance, MPC weights, Kalman filter noise
- `safety.fall_protection` / `fall_angle`, `debug_log_file` (per-tick CSV for tuning)
- `self_righting.*`: after a fall the robot waits until it is at rest, rolls itself back onto its
  belly with the legs, stands up and resumes the commanded mode (or stays in damping mode when
  `enabled: false` or after `max_attempts`)

Numeric parameters accept integers as well (`kp: 80` or `kp: 80.0`).

## Terrain perception
```bash
ros2 launch hyperdog_gazebo sim.launch.xml world:=stairs use_camera:=true perception:=true
```
`height_map_node` (package `hyperdog_perception`) fuses the depth camera point cloud into a
robot-centric elevation map (2 cm cells, 2.4 m x 2.4 m) in the odometry frame and publishes it on
`hyperdog/height_map`. The locomotion controller then takes foothold heights from the map, moves
footholds away from step edges, lifts the swing foot above the highest terrain along its path and
uses a lift - move - lower swing over steps (`terrain.*` in `locomotion.yaml`). With perception the
launch file also enables `estimation.foot_rolling` (leg odometry that models the rolling of the
spherical feet), because odometry drift directly shifts footholds relative to the map.

## Navigation (Nav2)
```bash
ros2 launch hyperdog_navigation sim_navigation.launch.xml headless:=true
ros2 run hyperdog_navigation navigate_test --ros-args -p use_sim_time:=true -p goal_x:=5.0
```
Navigation runs in the odometry frame with rolling costmaps built from the 3D lidar, the NavFn
planner and the MPPI controller with the omnidirectional motion model; its velocity commands go
through the velocity smoother to `/cmd_vel`. See [hyperdog_navigation](hyperdog_navigation/README.md).

## Validation in simulation
```bash
ros2 launch hyperdog_gazebo validate.launch.xml scenario:=full                    # default
ros2 launch hyperdog_gazebo validate.launch.xml scenario:=stress
ros2 launch hyperdog_gazebo validate.launch.xml scenario:=terrain world:=terrain
ros2 launch hyperdog_gazebo validate.launch.xml scenario:=fall                    # self-righting
# randomized robustness campaign: payload, foot friction, IMU noise, motor current / torque
# constant / friction and command latency are sampled per run
ros2 run hyperdog_gazebo robustness_campaign --runs 10 --seed 1 --scenario robust --out campaign
```
The `scenario_runner` drives the robot, applies pushes through Gazebo's `ApplyLinkWrench`
system and grades the run against the simulator ground truth. It then writes a markdown
report and a CSV time series, and exits non-zero on failure. Results of the current code
(headless, Gazebo Harmonic 8.10, default parameters):

| scenario | checks | result |
|---|---|---|
| [`full`](docs/validation/full.md) | stand, 40 N lateral and 50 N frontal pushes (0.2 s) while standing, trot 0.4 m/s, 40 N push while trotting, sideways 0.2 m/s, turn 0.8 rad/s, walk gait, backward, stop | **PASS** (max tilt 15 deg) |
| [`stress`](docs/validation/stress.md) | 70 N lateral and 80 N diagonal pushes (0.2 s) while standing, fast trot 0.6 m/s, 50 N push while trotting, trot + turn | **PASS** (9 of 11 runs, see limitations) |
| [`speed`](docs/validation/speed.md) | trot at 0.3 / 0.4 / 0.5 / 0.6 / 0.7 m/s, stop | **PASS** (0.7 m/s commanded -> 0.67 m/s) |
| [`terrain`](docs/validation/terrain.md) | trot over an 8 deg ramp, 0.28 m plateau and ramp down | **PASS** |
| [`rough`](docs/validation/rough.md) | trot 6 m over randomly placed 1-3 cm slabs | **PASS** |
| [`slippery`](docs/validation/slippery.md) | trot across a 3 m patch with friction coefficient 0.3 | **PASS** |
| [`fall`](docs/validation/fall.md) | knocked over by a roll torque, dropped on its side: self-right, stand up, trot again | **PASS** |
| [`navigation`](docs/validation/navigation.md) | Nav2 goal 5 m away through an obstacle course (lidar costmaps, MPPI) | **PASS** (true final error 0.10 m) |
| [`stairs`](docs/validation/stairs.md) (perception) | 5 steps up (4 cm), landing, 5 steps down | experimental: 1 of 5 runs complete |
| [`robust` campaign](docs/validation/robustness.md) | 10 randomized robots (payload 0-1.5 kg off-centre, foot mu 0.5-1.2, 1-3x IMU noise, motor current -20 %, Kt +-10 %, friction, 0-6 ms latency): 40 N push, trot, trot + turn, stop | **10 / 10 PASS** |

## Development
See [CONTRIBUTING.md](CONTRIBUTING.md) for the code organisation and conventions. `colcon test` runs
the unit tests and the linters (`ament_uncrustify`, `ament_cpplint`, `ament_lint_cmake`,
`ament_xmllint`). GitHub Actions builds and tests every push to `latest` and then runs the `full`
validation scenario headless in Gazebo (the report is attached to the workflow run).

## Real robot
```bash
sudo ip link set can0 up type can bitrate 1000000
ros2 launch hyperdog_bringup robot.launch.xml can_interface:=can0 imu_topic:=/imu/data
```
CAN IDs are 1-12 in joint order (FR, FL, BR, BL x hip, uleg, lleg). `can_id`, `direction` and `offset`
are joint `<param>`s in `hyperdog_description/urdf/ros2_control.xacro`. The MIT packing ranges
(`p_max`, `v_max`, `kp_max`, `kd_max`, `t_max`; defaults 12.5, 50, 500, 5, 25) can be added there as
additional joint params to match your drivers. On the robot `simulate_motor_dynamics` is false (the
drivers close the current loop), attitude comes from a Mahony filter and contacts are estimated from
the motor torques (`hyperdog_bringup/config/robot_overrides.yaml`). The robot waits for `START` on the
gamepad before standing up.

Bring the robot up in stages with the [hardware bring-up guide](docs/hardware_bringup.md): single
motor on the bench (`mit_can_probe` - scan, read, zero, low-gain hold), one joint at a time on a
stand (`joint_test` - sine tracking and torque step with current / saturation / temperature
summary), the whole robot in the air, then on the ground.

## Known limitations
- Validated speed envelope: 0.7 m/s trotting (commands are clamped there). Tracking is looser at
  0.6 m/s (0.49 m/s achieved) than at 0.5 or 0.7 m/s.
- The 80 N diagonal push while standing in the `stress` scenario is at the limit of what the controller
  recovers from (passed 9 of 11 runs; in the failing runs the body tilted 47-48 deg, beyond the
  46 deg threshold of the scenario). The simulation is not bit-for-bit deterministic, because ROS nodes and
  Gazebo run asynchronously, so results near the limits can vary between runs.
- Self-righting works from the side and from the belly-down tumbles a push produces. Lying exactly on
  its back, the +-1 rad hip range lets HyperDog kick itself onto its side, but it comes to rest leaning
  back and does not complete the roll (`scenario:=fall_back`). After `max_attempts` it stays in
  damping mode.
- Stairs (`stairs` world, 4 cm rise, 35 cm run) are experimental. Blind, the robot fails at the
  first step. With terrain perception, 1 of 5 runs went up and down the whole staircase, 2 reached
  the top landing and 2 fell on the first steps. The remaining error is odometry drift on the steps
  (3-7 cm): it misplaces footholds relative to the map. Registering the map with the foot contacts
  (`terrain.register_with_feet`) is implemented but not yet stable, so it is off by default.
- Leg odometry with the default `estimation.foot_rolling: false` under-estimates the travelled
  distance by about 10 % (the spherical feet roll). `foot_rolling: true` removes this bias, but the
  gait gains were tuned without it and the trot envelope then drops to about 0.5 m/s; the
  integral speed tracking meant to compensate (`locomotion.speed_integral_gain`) destabilised the
  trot at 0.5 m/s and is off. Retuning the gait for the unbiased estimator is open work.
- The `MitCanSystem` hardware interface and `mit_can_probe` are compiled and load, but they have not
  been tested on hardware yet. Follow the staged [hardware bring-up guide](docs/hardware_bringup.md).
- Lidar and camera need a render engine (GPU, or Mesa software rendering for headless use). The
  validation scenarios disable them.

## License
Licensed under the [Apache License, Version 2.0](LICENSE). The ROS 2 Foxy version on the
`foxy` branch was released under the MIT license.

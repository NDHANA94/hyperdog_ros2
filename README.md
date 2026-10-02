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
 gamepad / nav stack                                   Gazebo Harmonic (DART, 1 kHz)  or  real robot
        |  /cmd_vel, /hyperdog/command                        ^ effort            | joint states, IMU,
        v                                                     |                   | foot contacts
 +---------------------------- hyperdog_locomotion (C++, 500 Hz) ---------------------------------+
 |  attitude (IMU / Mahony) -> contact estimation -> kinematic Kalman filter (base + feet)        |
 |  gait scheduler (trot/walk/pace/bound/pronk, safe transitions, auto-stepping on pushes)       |
 |  foothold planner (Raibert + capture point) -> min-jerk swing trajectories                    |
 |  convex MPC (stepping, 100 Hz)  |  QP force distribution (standing)   + friction cones on     |
 |                                 |                                       the estimated terrain |
 |  stance: tau = -J^T R^T f   swing: cartesian impedance + leg gravity compensation            |
 +--------------------------------------------------------------------------------------------+
        |  hyperdog_msgs/MotorCommands  (q, dq, kp, kd, tau_ff per joint)
        v
 +----------------- hyperdog_bldc_control (ros2_control, 1 kHz) -----------------+
 |  tau = kp (q* - q) + kd (dq* - dq) + tau_ff  -> BLDC motor model:             |
 |  current limit + thermal derating, back-EMF torque/speed envelope,            |
 |  current-loop lag, gearbox efficiency + friction  -> joint effort             |
 |  real robot: MitCanSystem (SocketCAN, MIT protocol) hardware interface        |
 +-------------------------------------------------------------------------------+
```

All code in this branch is C++. Configuration is done with YAML parameter files and
XML launch files.

## Packages

| package | content |
|---|---|
| `hyperdog_msgs` | `MotorCommands`, `MotorStates`, `LocomotionCommand`, `LocomotionState` (+ legacy Foxy messages) |
| `hyperdog_description` | xacro model, **`config/bldc_motors.yaml`** (actuator parameters), `config/controllers.yaml`, sensors |
| `hyperdog_bldc_control` | `BldcImpedanceController` (ros2_control controller + BLDC motor model), `MitCanSystem` hardware interface, unit tests |
| `hyperdog_locomotion` | locomotion library (`*_core`), `locomotion_node`, **`config/locomotion.yaml`**, unit tests |
| `hyperdog_gazebo` | Gazebo Harmonic worlds (`flat`, `terrain`), ros_gz bridges, `sim.launch.xml`, `validate.launch.xml`, `scenario_runner` |
| `hyperdog_teleop` | gamepad teleop (`/joy` -> `/cmd_vel` + `/hyperdog/command`), mapping in `config/joy_xbox.yaml` |
| `hyperdog_bringup` | real-robot launch (`robot.launch.xml`), robot parameter overrides |

### Sensors (simulation)
| sensor | topic | notes |
|---|---|---|
| IMU (500 Hz, noisy) | `/hyperdog/imu` | orientation, gyro, accelerometer |
| 4 foot contact sensors | `/hyperdog/foot_contact/{FR,FL,BR,BL}` | `ros_gz_interfaces/Contacts` |
| 3D lidar, 16 beams, 360 deg | `/hyperdog/lidar/points` | `use_lidar:=true` (needs rendering) |
| RGB-D camera | `/hyperdog/camera/{image,depth_image,points,camera_info}` | `use_camera:=true` (needs rendering) |
| joint encoders / torques | `/joint_states`, `/bldc_controller/motor_states` | motor current, power, winding temperature, saturation |
| state estimate | `/odom`, TF `odom -> base_link`, `/hyperdog/state` | from the kinematic Kalman filter |
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
ros2 launch hyperdog_gazebo sim.launch.xml world:=terrain         # ramps and small steps
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

## Validation in simulation
```bash
ros2 launch hyperdog_gazebo validate.launch.xml scenario:=full                    # default
ros2 launch hyperdog_gazebo validate.launch.xml scenario:=stress
ros2 launch hyperdog_gazebo validate.launch.xml scenario:=terrain world:=terrain
```
The `scenario_runner` drives the robot, applies pushes through Gazebo's `ApplyLinkWrench`
system and grades the run against the simulator ground truth. It then writes a markdown
report and a CSV time series, and exits non-zero on failure. Results of the current code
(headless, Gazebo Harmonic 8.10, default parameters):

| scenario | checks | result |
|---|---|---|
| [`full`](docs/validation/full.md) | stand, 40 N lateral and 50 N frontal pushes (0.2 s) while standing, trot 0.4 m/s, 40 N push while trotting, sideways 0.2 m/s, turn 0.8 rad/s, walk gait, backward, stop | **PASS** (max tilt 10 deg) |
| [`stress`](docs/validation/stress.md) | 70 N lateral and 80 N diagonal pushes (0.2 s) while standing, fast trot, 50 N push while trotting, trot + turn | **PASS** |
| [`terrain`](docs/validation/terrain.md) | trot over an 8 deg ramp, 0.28 m plateau and ramp down | **PASS** |

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

## Known limitations
- Validated speed envelope: about 0.4 m/s trotting. Commands are clamped to 0.5 m/s, and above roughly
  0.45 m/s the achieved speed saturates (0.55 m/s commanded gives 0.42 m/s in the stress test) because
  the single-rigid-body MPC ignores the heavy legs (about 60 % of the mass).
- The `MitCanSystem` hardware interface is compiled and loads, but it has not been tested on hardware yet.
  Check directions, offsets and current limits with the robot lifted off the ground.
- Lidar and camera need a render engine (GPU, or Mesa software rendering for headless use). The
  validation scenarios disable them.
- `uros/` and `uros_pkg/` are leftovers of the Foxy micro-ROS firmware and are not used by this branch.

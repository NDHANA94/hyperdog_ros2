# hyperdog_gazebo

Gazebo Harmonic simulation and automated validation.

| path | content |
|---|---|
| `launch/sim.launch.xml` | Gazebo + robot + bridges + controllers + locomotion controller |
| `launch/validate.launch.xml` | headless simulation + `scenario_runner` (exits with the result) |
| `worlds/{flat,terrain,rough,stairs,slippery}.sdf` | worlds (named `hyperdog`, with the ApplyLinkWrench system for pushes): flat ground; 8 deg ramps and a plateau; random 1-3 cm slabs; 4 cm stairs up and down; a friction 0.3 patch |
| `config/bridge*.yaml` | ros_gz bridge configuration (core incl. the `set_pose` service, lidar, camera) |
| `src/scenarios.{hpp,cpp}` | scenario definitions (`full`, `push`, `walk`, `stress`, `speed`, `robust`, `fall`, `fall_back`, `terrain`, `rough`, `stairs`, `slippery`) |
| `src/scenario_runner.cpp` | drives the robot, applies pushes (forces and roll torques), places the robot (e.g. on its side), grades against ground truth, writes a report + CSV |
| `src/robustness_campaign.cpp` | runs a scenario N times with randomized robot / motor / sensor parameters and writes a pass-rate report |

Add a scenario by appending steps in `src/scenarios.cpp`; each step defines the command,
gait, pushes and the checks (velocity tracking window and tolerances, rest at the end, or for
`allow_fall` steps: standing upright again at the end).

`sim.launch.xml` and `validate.launch.xml` take model variations as arguments: `payload_mass`,
`payload_x`, `payload_y`, `foot_mu`, `imu_noise_scale` and `extra_controller_params` (a
ros2_control parameter file that overrides motor parameters).

```bash
ros2 run hyperdog_gazebo robustness_campaign --runs 20 --seed 7 --scenario robust --out campaign \
  --min-pass-rate 0.9      # exit code 1 below 90 %
```

# hyperdog_gazebo

Gazebo Harmonic simulation and automated validation.

| path | content |
|---|---|
| `launch/sim.launch.xml` | Gazebo + robot + bridges + controllers + locomotion controller |
| `launch/validate.launch.xml` | headless simulation + `scenario_runner` (exits with the result) |
| `worlds/flat.sdf`, `worlds/terrain.sdf` | worlds (named `hyperdog`, with the ApplyLinkWrench system for pushes) |
| `config/bridge*.yaml` | ros_gz bridge configuration (core, lidar, camera) |
| `src/scenarios.{hpp,cpp}` | scenario definitions (`full`, `push`, `walk`, `stress`, `terrain`) |
| `src/scenario_runner.cpp` | drives the robot, applies pushes, grades against ground truth, writes a report + CSV |

Add a scenario by appending steps in `src/scenarios.cpp`; each step defines the command,
gait, pushes and the checks (velocity tracking window and tolerances, rest at the end).

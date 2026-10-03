# Stairs with terrain perception (experimental)

`ros2 launch hyperdog_gazebo validate.launch.xml scenario:=stairs world:=stairs perception:=true`

5 runs: 1 went up and down the whole staircase (below), 2 reached the top landing, 2 fell on the
first steps. Without perception the robot fails at the first step. The run below then failed the
final "stop and stand" because the stop came while its feet were still on the last step edge;
the stairs step has since been lengthened so that the robot reaches flat ground first.


Distance travelled (ground truth): x = 5.58629 m, y = -0.641102 m

Scenario: `stairs` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.243164 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| trot up and down 4 cm stairs at 0.25 m/s | PASS | 24.478 | 0.238 / 0.461 | vx 0.25->0.22, vy 0.00->0.00, wz 0.00->0.01 | no | 0.063 |
| stop and stand | **FAIL** | 34.653 | 0.233 / 0.316 | end speed 0.081727 m/s | yes | 0.117 |

**Overall: FAIL**

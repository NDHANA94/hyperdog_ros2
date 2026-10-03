# HyperDog closed-loop validation report

Distance travelled (ground truth): x = -1.08508 m, y = -2.12892 m

Scenario: `fall` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.240275 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| stand | PASS | 1.557 | 0.240 / 0.241 |  | no | 0.001 |
| knocked over (roll torque) -> self-right + stand up | PASS | 174.486 | 0.017 / 0.315 | fell, end: LOCOMOTION tilt 1.4 deg | yes | 0.353 |
| trot forward 0.3 m/s after recovery | PASS | 4.095 | 0.237 / 0.259 | vx 0.30->0.32, vy 0.00->0.00, wz 0.00->-0.01 | no | 0.031 |
| dropped on its right side -> self-right + stand up | PASS | 97.867 | 0.017 / 0.261 | fell, end: LOCOMOTION tilt 1.7 deg | no | 0.255 |
| trot forward 0.3 m/s after recovery | PASS | 2.752 | 0.237 / 0.252 | vx 0.30->0.32, vy 0.00->0.00, wz 0.00->-0.00 | no | 0.029 |
| stop and stand | PASS | 2.334 | 0.238 / 0.252 | end speed 0.000407 m/s | no | 0.011 |

**Overall: PASS**

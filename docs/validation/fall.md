# HyperDog closed-loop validation report

Distance travelled (ground truth): x = -1.64034 m, y = -2.0126 m

Scenario: `fall` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.240003 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| stand | PASS | 1.537 | 0.240 / 0.241 |  | no | 0.001 |
| knocked over (roll torque) -> self-right + stand up | PASS | 179.308 | 0.017 / 0.292 | fell, end: LOCOMOTION tilt 1.6 deg | yes | 0.274 |
| trot forward 0.3 m/s after recovery | PASS | 3.740 | 0.237 / 0.259 | vx 0.30->0.32, vy 0.00->-0.00, wz 0.00->-0.00 | no | 0.031 |
| dropped on its right side -> self-right + stand up | PASS | 99.584 | 0.017 / 0.261 | fell, end: LOCOMOTION tilt 1.5 deg | no | 0.235 |
| trot forward 0.3 m/s after recovery | PASS | 3.505 | 0.238 / 0.256 | vx 0.30->0.33, vy 0.00->-0.00, wz 0.00->-0.00 | no | 0.030 |
| stop and stand | PASS | 2.995 | 0.238 / 0.254 | end speed 0.000523 m/s | no | 0.012 |

**Overall: PASS**

# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 3.56804 m, y = 1.38769 m

Scenario: `full` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.241218 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| stand still | PASS | 1.705 | 0.241 / 0.241 | end speed 0.000246 m/s | no | 0.002 |
| push while standing: lateral 40 N x 0.2 s | PASS | 10.601 | 0.235 / 0.282 | end speed 0.010721 m/s | yes | 0.137 |
| push while standing: frontal 50 N x 0.2 s | PASS | 3.355 | 0.236 / 0.254 | end speed 0.050431 m/s | yes | 0.024 |
| trot forward 0.4 m/s | PASS | 6.719 | 0.238 / 0.279 | vx 0.40->0.38, vy 0.00->-0.01, wz 0.00->0.01 | yes | 0.064 |
| push while trotting: lateral 40 N x 0.2 s | PASS | 10.636 | 0.238 / 0.286 |  | no | 0.150 |
| trot sideways 0.2 m/s | PASS | 3.748 | 0.240 / 0.256 | vx 0.00->-0.01, vy 0.20->0.22, wz 0.00->0.01 | no | 0.030 |
| turn in place 0.8 rad/s | PASS | 2.312 | 0.236 / 0.247 | vx 0.00->-0.05, vy 0.00->-0.02, wz 0.80->0.81 | no | 0.018 |
| walk gait forward 0.2 m/s | PASS | 5.178 | 0.236 / 0.245 | vx 0.20->0.22, vy 0.00->-0.00, wz 0.00->0.01 | no | 0.026 |
| trot backward -0.3 m/s | PASS | 2.988 | 0.237 / 0.246 | vx -0.30->-0.38, vy 0.00->-0.01, wz 0.00->0.01 | no | 0.038 |
| stop and stand | PASS | 1.845 | 0.237 / 0.245 | end speed 0.000384 m/s | no | 0.014 |

**Overall: PASS**

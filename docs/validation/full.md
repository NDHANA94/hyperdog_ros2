# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 3.63152 m, y = 1.35938 m

Scenario: `full` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.240044 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| stand still | PASS | 1.585 | 0.240 / 0.241 | end speed 0.000416 m/s | no | 0.001 |
| push while standing: lateral 40 N x 0.2 s | PASS | 9.794 | 0.236 / 0.285 | end speed 0.021145 m/s | yes | 0.150 |
| push while standing: frontal 50 N x 0.2 s | PASS | 4.440 | 0.236 / 0.257 | end speed 0.000372 m/s | yes | 0.017 |
| trot forward 0.4 m/s | PASS | 6.027 | 0.238 / 0.296 | vx 0.40->0.37, vy 0.00->-0.00, wz 0.00->0.00 | no | 0.089 |
| push while trotting: lateral 40 N x 0.2 s | PASS | 10.264 | 0.240 / 0.296 |  | no | 0.112 |
| trot sideways 0.2 m/s | PASS | 5.167 | 0.242 / 0.275 | vx 0.00->-0.01, vy 0.20->0.23, wz 0.00->-0.01 | no | 0.033 |
| turn in place 0.8 rad/s | PASS | 2.561 | 0.238 / 0.247 | vx 0.00->-0.05, vy 0.00->-0.02, wz 0.80->0.80 | no | 0.018 |
| walk gait forward 0.2 m/s | PASS | 5.437 | 0.238 / 0.244 | vx 0.20->0.22, vy 0.00->-0.01, wz 0.00->0.00 | no | 0.026 |
| trot backward -0.3 m/s | PASS | 3.950 | 0.236 / 0.245 | vx -0.30->-0.38, vy 0.00->-0.01, wz 0.00->0.01 | no | 0.036 |
| stop and stand | PASS | 1.719 | 0.237 / 0.244 | end speed 0.000616 m/s | no | 0.011 |

**Overall: PASS**

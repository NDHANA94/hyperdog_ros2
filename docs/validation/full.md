# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 4.1031 m, y = 1.19477 m

Scenario: `full` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.240126 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| stand still | PASS | 1.808 | 0.240 / 0.241 | end speed 0.000091 m/s | no | 0.002 |
| push while standing: lateral 40 N x 0.2 s | PASS | 6.648 | 0.236 / 0.255 | end speed 0.028508 m/s | yes | 0.022 |
| push while standing: frontal 50 N x 0.2 s | PASS | 2.909 | 0.237 / 0.256 | end speed 0.016448 m/s | yes | 0.018 |
| trot forward 0.4 m/s | PASS | 7.091 | 0.238 / 0.264 | vx 0.40->0.38, vy 0.00->0.05, wz 0.00->-0.03 | yes | 0.050 |
| push while trotting: lateral 40 N x 0.2 s | PASS | 15.114 | 0.228 / 0.284 |  | no | 0.057 |
| trot sideways 0.2 m/s | PASS | 5.666 | 0.245 / 0.260 | vx 0.00->-0.02, vy 0.20->0.18, wz 0.00->-0.07 | no | 0.032 |
| turn in place 0.8 rad/s | PASS | 4.830 | 0.237 / 0.255 | vx 0.00->-0.04, vy 0.00->-0.02, wz 0.80->0.79 | no | 0.019 |
| walk gait forward 0.2 m/s | PASS | 8.677 | 0.225 / 0.245 | vx 0.20->0.13, vy 0.00->0.07, wz 0.00->0.01 | no | 0.032 |
| trot backward -0.3 m/s | PASS | 6.140 | 0.229 / 0.247 | vx -0.30->-0.38, vy 0.00->-0.01, wz 0.00->0.01 | no | 0.037 |
| stop and stand | PASS | 2.667 | 0.238 / 0.249 | end speed 0.000653 m/s | no | 0.013 |

**Overall: PASS**

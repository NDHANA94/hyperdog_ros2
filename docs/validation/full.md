# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 4.06508 m, y = 1.19994 m

Scenario: `full` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.240729 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| stand still | PASS | 1.403 | 0.241 / 0.242 | end speed 0.000247 m/s | no | 0.002 |
| push while standing: lateral 40 N x 0.2 s | PASS | 7.012 | 0.233 / 0.250 | end speed 0.033715 m/s | yes | 0.026 |
| push while standing: frontal 50 N x 0.2 s | PASS | 4.146 | 0.238 / 0.257 | end speed 0.029461 m/s | yes | 0.022 |
| trot forward 0.4 m/s | PASS | 7.407 | 0.238 / 0.267 | vx 0.40->0.41, vy 0.00->0.01, wz 0.00->-0.01 | yes | 0.049 |
| push while trotting: lateral 40 N x 0.2 s | PASS | 15.107 | 0.231 / 0.271 |  | no | 0.058 |
| trot sideways 0.2 m/s | PASS | 5.886 | 0.248 / 0.269 | vx 0.00->-0.02, vy 0.20->0.18, wz 0.00->0.06 | no | 0.038 |
| turn in place 0.8 rad/s | PASS | 3.786 | 0.238 / 0.253 | vx 0.00->-0.05, vy 0.00->-0.02, wz 0.80->0.78 | no | 0.018 |
| walk gait forward 0.2 m/s | PASS | 7.982 | 0.227 / 0.246 | vx 0.20->0.14, vy 0.00->0.07, wz 0.00->0.00 | no | 0.030 |
| trot backward -0.3 m/s | PASS | 6.617 | 0.229 / 0.249 | vx -0.30->-0.38, vy 0.00->-0.00, wz 0.00->-0.01 | no | 0.035 |
| stop and stand | PASS | 1.989 | 0.238 / 0.247 | end speed 0.001070 m/s | no | 0.013 |

**Overall: PASS**

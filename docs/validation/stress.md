# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 4.18769 m, y = 1.94603 m

Scenario: `stress` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.239976 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| push while standing: lateral 70 N x 0.2 s | PASS | 15.703 | 0.234 / 0.281 | end speed 0.066886 m/s | yes | 0.045 |
| push while standing: diagonal 80 N x 0.2 s | PASS | 23.520 | 0.224 / 0.275 | end speed 0.026376 m/s | yes | 0.068 |
| fast trot 0.6 m/s | PASS | 21.117 | 0.232 / 0.288 | vx 0.60->0.56, vy 0.00->-0.01, wz 0.00->-0.06 | yes | 0.094 |
| push while fast trotting: lateral 50 N x 0.2 s | PASS | 12.324 | 0.231 / 0.272 |  | no | 0.091 |
| trot + turn 0.4 m/s, 0.6 rad/s | PASS | 6.815 | 0.244 / 0.269 | vx 0.40->0.38, vy 0.00->0.03, wz 0.60->0.61 | no | 0.054 |
| stop and stand | PASS | 4.239 | 0.239 / 0.266 | end speed 0.001060 m/s | no | 0.014 |

**Overall: PASS**

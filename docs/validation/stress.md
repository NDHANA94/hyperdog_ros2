# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 3.09516 m, y = 1.57315 m

Scenario: `stress` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.23975 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| push while standing: lateral 70 N x 0.2 s | PASS | 15.392 | 0.236 / 0.280 | end speed 0.039745 m/s | yes | 0.163 |
| push while standing: diagonal 80 N x 0.2 s | PASS | 11.966 | 0.188 / 0.258 | end speed 0.008943 m/s | yes | 0.151 |
| fast trot 0.55 m/s | PASS | 6.444 | 0.237 / 0.297 | vx 0.55->0.42, vy 0.00->0.01, wz 0.00->-0.03 | yes | 0.119 |
| push while fast trotting: lateral 50 N x 0.2 s | PASS | 14.035 | 0.232 / 0.287 |  | no | 0.139 |
| trot + turn 0.4 m/s, 0.6 rad/s | PASS | 6.739 | 0.247 / 0.298 | vx 0.40->0.38, vy 0.00->0.03, wz 0.60->0.59 | no | 0.096 |
| stop and stand | PASS | 4.849 | 0.237 / 0.261 | end speed 0.000810 m/s | no | 0.014 |

**Overall: PASS**

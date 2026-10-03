# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 3.94304 m, y = 1.18475 m

Scenario: `stress` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.24014 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| push while standing: lateral 70 N x 0.2 s | PASS | 10.475 | 0.231 / 0.275 | end speed 0.057386 m/s | yes | 0.055 |
| push while standing: diagonal 80 N x 0.2 s | PASS | 22.199 | 0.206 / 0.271 | end speed 0.013413 m/s | yes | 0.095 |
| fast trot 0.6 m/s | PASS | 8.807 | 0.237 / 0.283 | vx 0.60->0.53, vy 0.00->-0.05, wz 0.00->-0.03 | yes | 0.071 |
| push while fast trotting: lateral 50 N x 0.2 s | PASS | 9.733 | 0.235 / 0.283 |  | no | 0.096 |
| trot + turn 0.4 m/s, 0.6 rad/s | PASS | 8.263 | 0.243 / 0.266 | vx 0.40->0.37, vy 0.00->0.04, wz 0.60->0.59 | no | 0.055 |
| stop and stand | PASS | 4.371 | 0.238 / 0.265 | end speed 0.000172 m/s | no | 0.015 |

**Overall: PASS**

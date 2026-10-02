# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 3.04181 m, y = 1.45756 m

Scenario: `stress` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.241807 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| push while standing: lateral 70 N x 0.2 s | PASS | 11.317 | 0.236 / 0.278 | end speed 0.052292 m/s | yes | 0.082 |
| push while standing: diagonal 80 N x 0.2 s | PASS | 10.550 | 0.198 / 0.258 | end speed 0.005398 m/s | yes | 0.136 |
| fast trot 0.55 m/s | PASS | 8.679 | 0.237 / 0.301 | vx 0.55->0.40, vy 0.00->-0.02, wz 0.00->0.01 | yes | 0.143 |
| push while fast trotting: lateral 50 N x 0.2 s | PASS | 8.522 | 0.246 / 0.301 |  | no | 0.203 |
| trot + turn 0.4 m/s, 0.6 rad/s | PASS | 10.886 | 0.243 / 0.303 | vx 0.40->0.36, vy 0.00->0.05, wz 0.60->0.60 | no | 0.162 |
| stop and stand | PASS | 6.537 | 0.239 / 0.264 | end speed 0.000527 m/s | no | 0.023 |

**Overall: PASS**

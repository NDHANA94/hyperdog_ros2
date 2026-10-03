# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 11.839 m, y = -0.0426525 m

Scenario: `speed` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.240011 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| trot 0.3 m/s | PASS | 2.876 | 0.237 / 0.251 | vx 0.30->0.33, vy 0.00->0.00, wz 0.00->-0.01 | no | 0.031 |
| trot 0.4 m/s | PASS | 6.362 | 0.243 / 0.262 | vx 0.40->0.40, vy 0.00->0.02, wz 0.00->-0.04 | no | 0.046 |
| trot 0.5 m/s | PASS | 8.455 | 0.243 / 0.273 | vx 0.50->0.47, vy 0.00->0.01, wz 0.00->-0.08 | no | 0.065 |
| trot 0.6 m/s | PASS | 9.233 | 0.233 / 0.274 | vx 0.60->0.49, vy 0.00->0.03, wz 0.00->-0.06 | no | 0.084 |
| trot 0.7 m/s | PASS | 15.793 | 0.236 / 0.285 | vx 0.70->0.67, vy 0.00->-0.06, wz 0.00->0.15 | no | 0.087 |
| stop and stand | PASS | 7.012 | 0.232 / 0.281 | end speed 0.000807 m/s | no | 0.022 |

**Overall: PASS**

# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 11.6416 m, y = 0.715678 m

Scenario: `speed` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.240742 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| trot 0.3 m/s | PASS | 2.940 | 0.237 / 0.252 | vx 0.30->0.33, vy 0.00->0.00, wz 0.00->-0.01 | no | 0.030 |
| trot 0.4 m/s | PASS | 6.033 | 0.243 / 0.261 | vx 0.40->0.40, vy 0.00->-0.02, wz 0.00->-0.02 | no | 0.041 |
| trot 0.5 m/s | PASS | 10.323 | 0.244 / 0.273 | vx 0.50->0.45, vy 0.00->0.02, wz 0.00->-0.12 | no | 0.069 |
| trot 0.6 m/s | PASS | 11.678 | 0.226 / 0.287 | vx 0.60->0.55, vy 0.00->-0.00, wz 0.00->0.10 | no | 0.081 |
| trot 0.7 m/s | PASS | 9.066 | 0.220 / 0.274 | vx 0.70->0.61, vy 0.00->0.00, wz 0.00->-0.05 | no | 0.093 |
| stop and stand | PASS | 7.593 | 0.238 / 0.267 | end speed 0.000866 m/s | no | 0.022 |

**Overall: PASS**

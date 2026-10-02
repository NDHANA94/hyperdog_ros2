# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 9.73899 m, y = 1.14582 m

Scenario: `terrain` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.239952 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| trot over 8 deg ramp, plateau and ramp down at 0.3 m/s | PASS | 18.984 | 0.235 / 0.571 | vx 0.30->0.31, vy 0.00->-0.00, wz 0.00->0.01 | no | 0.208 |
| stop and stand on flat ground | PASS | 2.213 | 0.238 / 0.251 | end speed 0.000555 m/s | no | 0.012 |

**Overall: PASS**

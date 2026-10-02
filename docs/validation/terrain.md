# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 9.64076 m, y = -0.399422 m

Scenario: `terrain` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.239718 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| trot over 8 deg ramp, plateau and ramp down at 0.3 m/s | PASS | 26.400 | 0.236 / 0.550 | vx 0.30->0.30, vy 0.00->0.00, wz 0.00->-0.00 | no | 0.165 |
| stop and stand on flat ground | PASS | 2.552 | 0.237 / 0.251 | end speed 0.000431 m/s | no | 0.012 |

**Overall: PASS**

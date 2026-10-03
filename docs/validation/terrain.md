# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 10.284 m, y = 0.056412 m

Scenario: `terrain` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.239949 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| trot over 8 deg ramp, plateau and ramp down at 0.3 m/s | PASS | 15.514 | 0.237 / 0.533 | vx 0.30->0.32, vy 0.00->-0.00, wz 0.00->-0.00 | no | 0.036 |
| stop and stand on flat ground | PASS | 3.883 | 0.237 / 0.255 | end speed 0.000680 m/s | no | 0.011 |

**Overall: PASS**

# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 6.42967 m, y = -0.00174907 m

Scenario: `slippery` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.240792 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| trot across a slippery patch (mu 0.3) at 0.3 m/s | PASS | 3.809 | 0.238 / 0.252 | vx 0.30->0.33, vy 0.00->0.00, wz 0.00->0.00 | no | 0.028 |
| stop and stand | PASS | 3.589 | 0.238 / 0.254 | end speed 0.000465 m/s | no | 0.011 |

**Overall: PASS**

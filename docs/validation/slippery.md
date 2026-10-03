# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 6.42848 m, y = 0.00168375 m

Scenario: `slippery` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.240104 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| trot across a slippery patch (mu 0.3) at 0.3 m/s | PASS | 3.922 | 0.237 / 0.251 | vx 0.30->0.33, vy 0.00->0.00, wz 0.00->0.00 | no | 0.028 |
| stop and stand | PASS | 2.748 | 0.237 / 0.254 | end speed 0.000956 m/s | no | 0.011 |

**Overall: PASS**

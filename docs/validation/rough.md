# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 7.62855 m, y = 0.23847 m

Scenario: `rough` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.239983 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| trot over rough ground (1-3 cm) at 0.3 m/s | PASS | 14.993 | 0.237 / 0.292 | vx 0.30->0.30, vy 0.00->0.01, wz 0.00->-0.01 | no | 0.043 |
| stop and stand | PASS | 10.514 | 0.236 / 0.265 | end speed 0.001471 m/s | no | 0.024 |

**Overall: PASS**

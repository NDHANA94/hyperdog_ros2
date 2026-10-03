# HyperDog closed-loop validation report

Distance travelled (ground truth): x = 7.46755 m, y = 0.439687 m

Scenario: `rough` - simulator: Gazebo Harmonic (DART, 1 kHz), actuators: simulated BLDC (MIT impedance mode, 1 kHz), controller: hyperdog_locomotion (500 Hz)

Standing height after stand-up: 0.241277 m

| step | result | max tilt [deg] | height min/max [m] | tracking (cmd -> achieved) | auto-step | est. vel RMSE [m/s] |
|---|---|---|---|---|---|---|
| trot over rough ground (1-3 cm) at 0.3 m/s | PASS | 23.153 | 0.236 / 0.292 | vx 0.30->0.29, vy 0.00->0.01, wz 0.00->-0.00 | no | 0.048 |
| stop and stand | PASS | 10.261 | 0.243 / 0.274 | end speed 0.001947 m/s | no | 0.020 |

**Overall: PASS**

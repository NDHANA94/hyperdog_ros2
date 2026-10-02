# hyperdog_msgs

| message | purpose |
|---|---|
| `MotorCommands` | MIT-mode command per joint (position, velocity, kp, kd, feed-forward torque) |
| `MotorStates` | actuator feedback (torque, current, power, winding temperature, saturation) |
| `LocomotionCommand` | mode (passive / stand / locomotion / sit), gait, body height, step height, body attitude |
| `LocomotionState` | controller diagnostics (mode, gait, contacts, phases, forces, estimated state) |

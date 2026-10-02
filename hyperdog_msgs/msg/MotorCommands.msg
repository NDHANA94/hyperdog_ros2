# Command for a set of BLDC actuators in "MIT / impedance" mode.
# For every joint i the low-level controller applies
#   tau_i = kp_i * (position_i - q_i) + kd_i * (velocity_i - dq_i) + effort_i
# and then passes the request through the BLDC motor model
# (current limit, back-EMF torque/speed envelope, current-loop lag, friction).
# Arrays must all have the same length as `name` (empty kp/kd => use defaults).
std_msgs/Header header
string[] name
float64[] position   # desired joint position [rad]
float64[] velocity   # desired joint velocity [rad/s]
float64[] effort     # feed-forward joint torque [Nm]
float64[] kp         # joint stiffness [Nm/rad]
float64[] kd         # joint damping [Nm s/rad]

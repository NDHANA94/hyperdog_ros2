# Feedback from the BLDC actuators (simulated motor model or real drivers).
std_msgs/Header header
string[] name
float64[] position              # joint position [rad]
float64[] velocity              # joint velocity [rad/s]
float64[] torque                # delivered output torque [Nm]
float64[] torque_command        # torque requested by the impedance law [Nm]
float64[] current               # q-axis phase current [A]
float64[] electrical_power      # [W]
float64[] winding_temperature   # estimated winding temperature [degC]
bool[] saturated                # torque was clipped by the current / voltage / thermal limit

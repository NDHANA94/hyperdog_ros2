# Diagnostic state published by the HyperDog locomotion controller.
std_msgs/Header header
string mode
string gait
string balance_controller        # "mpc" or "qp"
bool[4] contact                  # estimated contact state, order FR, FL, BR, BL
bool[4] scheduled_contact        # contact state requested by the gait scheduler
float64[4] phase                 # per-leg gait phase [0, 1)
float64[12] foot_force           # desired ground reaction forces (world frame) FR xyz, FL xyz, ...
geometry_msgs/Vector3 rpy        # estimated body roll, pitch, yaw [rad]
geometry_msgs/Vector3 linear_velocity   # estimated body velocity (world frame) [m/s]
geometry_msgs/Vector3 angular_velocity  # body angular velocity (body frame) [rad/s]
float64 body_height              # estimated base height above the stance feet [m]
bool disturbance_recovery        # true while auto-stepping to reject a disturbance
float64 solve_time_ms            # last balance-controller solve time

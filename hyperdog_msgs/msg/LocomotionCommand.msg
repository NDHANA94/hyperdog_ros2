# High level command for the HyperDog locomotion controller.
# Linear/angular velocity is commanded separately through geometry_msgs/Twist on /cmd_vel.
uint8 MODE_PASSIVE = 0      # motors in damping mode, robot goes limp
uint8 MODE_STAND = 1        # stand up and balance (closed loop, auto-stepping on pushes)
uint8 MODE_LOCOMOTION = 2   # walk according to /cmd_vel with the selected gait
uint8 MODE_SIT = 3          # lower the body and go passive

uint8 mode
string gait                 # "trot", "walk", "pace", "bound", "pronk", "stand"; empty = keep current
float64 body_height         # [m] nominal base height, <= 0 => keep current
float64 step_height         # [m] swing apex height, <= 0 => keep current
geometry_msgs/Vector3 body_rpy   # [rad] body orientation offset (roll, pitch, yaw) while standing

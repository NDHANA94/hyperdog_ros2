# std_msgs/Header header


# states represent the robot's states
# states[0] start
# states[1] walk
# states[2] side_move_mode
bool[3] states       

# This represent the type of selected gait: 0,1,2,3,....
uint8 gait_type 0    

# This represents the robot pose
# pose.position: slant-x, slant_y, height 
# pose.orientation: roll, pitch, yaw
geometry_msgs/Pose pose   

# This represents the gait_step commands
# gait_step.x = steplen_x
# gait_step.y = steplen_y
# gait_step.z = swing_height
geometry_msgs/Vector3 gait_step





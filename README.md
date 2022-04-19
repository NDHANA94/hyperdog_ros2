# hyperdog_ros2

## Contains
This repository contains ros2 packages for quadruped robot Hyperdog.
packages are :
  1. **`hyperdog_msgs`** : this package contains the msgs those used by other packages.
  
        1. `JoyCtrlCmds` : this contains the control variables of the robot from the gamepad
              - `bool[3] states` : { start, walk, side_move_mode} 
              - `uint8 gait_type` : to change the gait type
              - `geometry_msgs/Pose pose` : to control slant(x,y) and roll,pitch,yaw
              - `geometry_msgs/Vector3 gait_step` : gait_step.x = steplen_x, gait_step.y = steplen_y, gait_step.z = swing_height
              
        2. `Geometry`: this contains the parameters for cordinate of each leg and body orientation(roll,pitch,yaw)
              - `geometry_msgs/Point32 fr` : x,y,z end effector coordinates of FR leg
              - `geometry_msgs/Point32 fl` : x,y,z end effector coordinates of FL leg
              - `geometry_msgs/Point32 br` : x,y,z end effector coordinates of BR leg
              - `geometry_msgs/Point32 bl` : x,y,z end effector coordinates of BL leg
              - `geometry_msgs/Quaternion euler_ang` : roll, pitch, yaw angles
              
  2. **`hyperdog_teleop`** : this pkg creates `/hyperdog_teleop_gamepad_node`. 
        - Node 1 : `/joy_node`
            This node creates commands to robot from Gamepad commands
            - subscriber : `/joy_node` 
            - publisher : `/hyperdog_joy_ctrl_cmd` using the interface `hyperdog_msgs/msg/JoyCtrlCmd`

  3. **`hyperdog_ctrl`** : This pkg has `Body_motion_planner` and `gait_generater` and creates the nodes `/command_manager_node` and `/IK_node`
        - `Body_motion_planner` : plans body motions from control commands comes from `/command_manager_node`
        - `gait_generator` : generates gaits acording to the given gait_type command from the Gamepad 
   
        - Node 1 : `/command_manager_node` 
            - subscriber : `/hyperdog_joy_ctrl_cmd` via `hyperdog_msgs/msg/JoyCtrlCmds` interface
            - publisher : `/hyperdog_geometry` via `hyperdog_msgs/msg/Geometry` interface

        - Node 2 : `/IK_node`
            - subscriber : `/hyperdog_geometry` via `hyperdog_msgs/msg/Geometry` interface
            - publisher  : `/hyperdog_jointController/commands`
  
  4. **`uros`** : this is the micro_ros package from its official git. this package is used to launch `micro_ros_agent` to communicate with micro-controllers which run micro_ros via ROS2


## Building

 Create a ROS2 workspace and build this package for ROS2 foxy
 'mkdir hyperdog_ws/src
 cd hyperdog_ws/src
 git clone 
            
        

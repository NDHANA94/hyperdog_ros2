# __________________________________________________________________________________
# MIT License                                                                       |
#                                                                                   |
# Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi                               |
#                                                                                   | 
# Permission is hereby granted, free of charge, to any person obtaining a copy      |
# of this software and associated documentation files (the "Software"), to deal     |
# in the Software without restriction, including without limitation the rights      |
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell         |
# copies of the Software, and to permit persons to whom the Software is             |
# furnished to do so, subject to the following conditions:                          |
#                                                                                   |
# The above copyright notice and this permission notice shall be included in all    |
# copies or substantial portions of the Software.                                   |
#                                                                                   |
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR        |
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,          |
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE       |
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER            |
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,     |
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     |
# SOFTWARE.                                                                         |
# __________________________________________________________________________________|
 
import os

from black import out
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

import xacro

# configure robot's urdf file
pkg_hyperdog_gazebo = 'hyperdog_gazebo_sim'
robot_description_subpath = 'description/hyperdog.urdf.xacro'
xacro_file = os.path.join(get_package_share_directory(pkg_hyperdog_gazebo),robot_description_subpath)
robot_description_raw = xacro.process_file(xacro_file).toxml()

#configure gazebo
pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros') 
pkg_hyperdog_gazebo = FindPackageShare(package='hyperdog_gazebo_sim').find('hyperdog_gazebo_sim')

# Set the path to the world file
world_file_name = 'contact.world'
world_path = os.path.join(pkg_hyperdog_gazebo, 'worlds', world_file_name)
# world_path = os.path.join(pkg_hyperdog_gazebo, 'worlds', 'contact.world')

# world_path = "/usr/share/gazebo-11/worlds/friction_demo.world"


# configure hyperdog teleop
teleop_pkg_name = 'hyperdog_teleop'
teleop_launch_file = "/hyperdog_teleop.launch.py"

# set the controller
gazebo_controller = 'hyperdog_joint_controller'
 

 
def generate_launch_description():

  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value='empty.world',#world_path,
    description='Full path to the world model file to load')

  declare_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description_raw,
                    'use_sim_time':True}])
  
  hyperdog_gz_joint_ctrl_node = Node(
    package='hyperdog_gazebo_sim',
    executable='hyperdog_gazebo_joint_ctrl_node',
    output='screen'
  )
    
  spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', 
                    arguments=['-topic', 'robot_description',
                               '-entity', 'HyperDog'],
                    output='screen')

  load_joint_state_controller = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['joint_state_broadcaster'],
      output='screen',
  )
  
  load_forward_command_controller = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['gazebo_joint_controller'],
      output='screen',
  )



  # Specify the actions
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
 

  # Create the launch description and populate
  return  LaunchDescription([    

    RegisterEventHandler(
      event_handler=OnProcessExit(
        target_action=spawn_entity,
        on_exit=[load_joint_state_controller],
      )
    ),
    RegisterEventHandler(
      event_handler=OnProcessExit(
        target_action=load_joint_state_controller,
        on_exit=[load_forward_command_controller],
      )
    ),
    declare_simulator_cmd,
    declare_use_sim_time_cmd,
    declare_use_simulator_cmd,
    declare_world_cmd,

    start_gazebo_server_cmd,
    start_gazebo_client_cmd,

    spawn_entity,
    declare_robot_state_publisher,
    # load_joint_state_controller,
    # laod_forward_command_controller,
    hyperdog_gz_joint_ctrl_node,
  
 ])
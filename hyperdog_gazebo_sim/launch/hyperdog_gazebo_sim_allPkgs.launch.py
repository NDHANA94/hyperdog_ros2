import os

from click import argument
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


# configure robot's urdf file
pkg_hyperdog_gazebo = 'hyperdog_gazebo_sim'
robot_description_subpath = 'description/hyperdog.urdf.xacro'
xacro_file = os.path.join(get_package_share_directory(pkg_hyperdog_gazebo),robot_description_subpath)
robot_description_raw = xacro.process_file(xacro_file).toxml()

teleop_pkg_name = 'hyperdog_teleop'
teleop_launch_file = "/hyperdog_teleop.launch.py"

gazebo_controller = 'hyperdog_joint_controller'

  # Set the path to this package.
pkg_share = FindPackageShare(package='hyperdog_gazebo_sim').find('hyperdog_gazebo_sim')

# Set the path to the world file
world_file_name = 'test_world.world'
world_path = os.path.join(pkg_share, 'worlds', world_file_name)

def generate_launch_description():

        
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        )
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description_raw,
                    'use_sim_time':True}])

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', 
                    arguments=['-topic', 'robot_description',
                               '-entity', 'HyperDog'],
                    output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
            'joint_state_broadcaster'],
        output='screen' )

    laod_forward_command_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 
            'gazebo_joint_controller'],
        output='screen'
    )

    launch_hyperdog_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(teleop_pkg_name), 'launch'), teleop_launch_file]),)

    launch_joy_cmd_executor = ExecuteProcess(
        cmd=['ros2', 'run', 'hyperdog_control', 'ctrl_cmd_executor'],
        output='screen'
    )

    hyperdog_joint_controller = Node(
        package='hyperdog_control',
        executable='hyperdog_joint_controller_node',
        output='screen'
    )

   
   

    return LaunchDescription([ 
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_controller,
        laod_forward_command_controller,
        launch_hyperdog_teleop,
        launch_joy_cmd_executor,
        hyperdog_joint_controller,

    ])

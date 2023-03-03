'''  
=========================================
    * Author: nipun.dhananjaya@gmail.com
    * Created: 04.02.2023
=========================================
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def with_xacro():
    pass

def generate_launch_description():

    """ With xacro files """
    hyperdog_ign_pkg = 'hyperdog_ign_sim'
    xacro_path = 'xacros/hyperdog.urdf.xacro'
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    
    xacro_file = os.path.join(get_package_share_directory(hyperdog_ign_pkg), xacro_path)

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    # robot_description = xacro.process_file(xacro_file).toxml() #{'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'hyperdog'],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_trajectory_controller'],
        output='screen'
    )



    return LaunchDescription([
        # launch gazebo environment
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', 'empty.sdf')]
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        node_robot_state_publisher,
        ignition_spawn_entity,
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        )
        
    ])
    

    # + 
    #                         ' -v 2 --gui-config ' + 
    #                         os.path.join(pkg_hyperdog_ign, 'ign', 'gui.config'), ''

    """
    
    # With model.sdf file

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_hyperdog_ign = get_package_share_directory('hyperdog_ign_sim')

    # gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
    )

    # spawn hyperdog
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'hyperdog',
                    '-x', '5.0',
                    '-z', '0.46',
                    '-y', '1.57',
                    '-file', os.path.join(pkg_hyperdog_ign, 'models', 'hyperdog', 'model.sdf')],
                output='screen')
    
    #  Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/hyperdog/joint/FR_hip_joint/0/cmd_pos@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ign_args',
            default_value=[os.path.join(pkg_hyperdog_ign, 'worlds', 'empty_world.sdf') ],
            description='Ignition Gazebo arguments'),
        gazebo,
        spawn,
        bridge
    ])

    """
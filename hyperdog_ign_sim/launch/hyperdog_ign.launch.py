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

def generate_launch_description():
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
        arguments=['/model/tracer/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ign_args',
            default_value=[os.path.join(pkg_hyperdog_ign, 'worlds', 'empty_world.sdf') + 
                            ' -v 2 --gui-config ' + 
                            os.path.join(pkg_hyperdog_ign, 'ign', 'gui.config'), ''],
            description='Ignition Gazebo arguments'),
        gazebo,
        spawn,
        bridge
    ])
    
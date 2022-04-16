import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    node_joy = ExecuteProcess(
        cmd=['ros2', 'run', 'joy', 'joy_node'],
        output='screen'
    )

    node_hyperdog_teleop_joy = ExecuteProcess(
        cmd=['ros2', 'run', 'hyperdog_teleop', 'hyperdog_teleop_joy_node'],
        output='screen'
    )

    node_hyperdog_ctrl = ExecuteProcess(
        cmd=['ros2', 'run', 'hyperdog_ctrl', 'cmd_manager_node'],
        output='screen'
    )

    node_IK_node = ExecuteProcess(
        cmd=['ros2', 'run', 'hyperdog_ctrl', 'IK_node'],
        output='screen'
    )

    node_uros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '-b', '115200', '--dev', '/dev/ttyUSB0'],
        output='screen'
    )

    hyperdog_gazebo_joint_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'hyperdog_gazebo_joint_cmd', 'hyperdog_gazebo_joint_controller'],
        output='screen'
    )


    return LaunchDescription([
        node_joy,
        node_hyperdog_teleop_joy,
        node_hyperdog_ctrl,
        node_IK_node,
        node_uros_agent,
        # hyperdog_gazebo_joint_cmd,
    ])

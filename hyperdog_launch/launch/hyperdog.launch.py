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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():

    node_joy = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    node_hyperdog_teleop_joy = Node(
        package='hyperdog_teleop',
        executable='hyperdog_teleop_joy_node',
        output='screen'
    )

    node_hyperdog_ctrl = Node(
        package='hyperdog_ctrl',
        executable='cmd_manager_node',
        output='screen'
    )

    node_IK_node = Node(
        package='hyperdog_ctrl',
        executable='IK_node',
        output='screen'
    )

    node_uros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '-b', '115200', '--dev', '/dev/ttyUSB0'],
        output='screen'
    )


    return LaunchDescription([
        node_joy,
        node_hyperdog_teleop_joy,

        RegisterEventHandler(
            OnProcessStart(
            target_action=node_hyperdog_teleop_joy,
            on_start=[
                    LogInfo(msg='node_hyperdog_teleop_joy started, starting hyperdog_ctrl'),
                    node_hyperdog_ctrl,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
            target_action=node_hyperdog_ctrl,
            on_start=[
                    LogInfo(msg='hyperdog_ctrl started, starting IK_node'),
                    node_IK_node
                ]
            )
        ),
        # RegisterEventHandler(
        #     OnProcessStart(
        #     target_action=node_IK_node,
        #     on_start=[
        #             LogInfo(msg='IK_node started, starting uros agent'),
        #             node_uros_agent
        #         ]
        #     )
        # ),
        # RegisterEventHandler(
        #     OnShutdown(
        #         on_shutdown=[LogInfo(
        #             msg=['Launch was asked to shutdown: ',
        #                 LocalSubstitution('event.reason')]
        #         )]
        #     )
        # ),
        # node_joy,
        # node_hyperdog_teleop_joy,
        # node_hyperdog_ctrl,
        # node_IK_node,
        # node_uros_agent,
    ])

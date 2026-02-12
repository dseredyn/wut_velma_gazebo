#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool, Hyungyu Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,\
    RegisterEventHandler, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_wut_velma_gazebo = get_package_share_directory('wut_velma_gazebo')
    launch_file_dir = os.path.join(pkg_wut_velma_gazebo, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    verbose = LaunchConfiguration('verbose')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_velma_description = get_package_share_directory('velma_description')
    pkg_velma_description_parent = os.path.dirname(pkg_velma_description)

    world = os.path.join(
        pkg_wut_velma_gazebo,
        'worlds',
        'empty_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world, TextSubstitution(text=' -v '), verbose], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Do not use URDF spawn (bug: https://github.com/gazebosim/gz-sim/issues/3261)
    # spawn_velma_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'spawn_velma.launch.py')
    #     )
    # )



    bridge_params = os.path.join(
        pkg_wut_velma_gazebo,
        'params',
        'velma_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    # TODO:
    arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arms_torso_controller", "-c", "/controller_manager"],
        output="screen",
    )

    cmd_kill_ruby = (
        f'echo "killing all ruby processes to terminate Gazebo server"'
        f' && pkill -9 ruby'
    )
    kill_ruby = ExecuteProcess(
            cmd=["bash", "-lc", cmd_kill_ruby],
            output="screen",
        )

    # Add the commands to the launch description
    startup_actions = [
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[PathJoinSubstitution([pkg_velma_description_parent])]
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[ PathJoinSubstitution([pkg_wut_velma_gazebo, 'models'])]
        ),
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[PathJoinSubstitution([pkg_velma_description_parent])]
        ),
        DeclareLaunchArgument('verbose', default_value='4',
                              description='Gazebo verbosity level (0-4).'),
        gzserver_cmd,
        gzclient_cmd,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_image_bridge_cmd,
        robot_state_publisher_cmd,
        jsb,
        # spawn_velma_cmd, # Do not use URDF spawn (bug: https://github.com/gazebosim/gz-sim/issues/3261)
    ]

    ld = LaunchDescription()

    ld.add_action(kill_ruby)

    ld.add_action(
        RegisterEventHandler(
        OnProcessExit(
            target_action=kill_ruby,
            on_exit=startup_actions,
        ))
    )

    return ld

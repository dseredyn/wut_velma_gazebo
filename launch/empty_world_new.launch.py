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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,\
    RegisterEventHandler, IncludeLaunchDescription, AppendEnvironmentVariable, LogInfo, \
    OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    cmd_kill_ruby = (
        f'echo "killing all ruby processes to terminate Gazebo server"'
        f' && pkill -9 ruby'
    )
    kill_ruby = ExecuteProcess(
            cmd=["bash", "-lc", cmd_kill_ruby],
            output="screen",
        )

    start_system = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wut_velma_gazebo'),
                'launch',
                'start_gazebo.launch.xml'
            ])
        )
    )

    spawn_velma = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wut_velma_gazebo'),
                'launch',
                'spawn_velma.launch.py'
            ])
        )
    )

    pkg_velma_moveit_config = get_package_share_directory('velma_moveit_config')
    default_xacro = PathJoinSubstitution([pkg_velma_moveit_config, 'config', 'velma.urdf.xacro'])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value='true',
            description="Use sim time",
        ),
        DeclareLaunchArgument(
            "xacro_file", default_value=default_xacro,
            description="Absolute path to URDF Xacro file.",
        ),
        kill_ruby,

        RegisterEventHandler(
            OnProcessExit(
                target_action=kill_ruby,
                on_exit=[
                    LogInfo(msg='Proces przygotowawczy zakończony, uruchamiam grupę węzłów...'),
                    start_system,
                ]
            )
        ),

        spawn_velma
    ])

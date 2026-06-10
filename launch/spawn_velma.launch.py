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
import subprocess
import shlex
import xacro

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


def generate_sdf_and_spawn(context, *args, **kwargs):
    xacro_file = LaunchConfiguration("xacro_file").perform(context)
    #xacro_args = LaunchConfiguration("xacro_args").perform(context).strip()
    sdf_out    = '/tmp/wut_velma.sdf'
    urdf_out = '/tmp/wut_velma.urdf'
    robot_name = 'velma'

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    # 1. urdf.xacro -> URDF
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "use_sim_time": use_sim_time,
        },
    )

    urdf_xml = doc.toprettyxml(indent="  ")

    with open(urdf_out, "w") as f:
        f.write(urdf_xml)

    # 2. URDF -> SDF
    result = subprocess.run(
        [
            "gz",
            "sdf",
            "-p",
            str(urdf_out),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    # Save the generated SDF
    with open(sdf_out, "w") as f:
        f.write(result.stdout)

    result = subprocess.run(
        [
            'sed',
            '--follow-symlinks',
            '-i',
            f's/<model name=\'velma\'>/<model name=\'velma\'>\\n    <self_collide>true<\\/self_collide>/g',
            f'{sdf_out}',
        ],
        check=True,
        # capture_output=True,
        text=True,
    )

    # f'sed --follow-symlinks -i "s/<model name=\'velma\'>/<model name=\'velma\'>\\n    <self_collide>true<\\/self_collide>/g" {sdf_out}'
    
    print(f"[launch] Generated URDF: {urdf_out}")
    print(f"[launch] Generated SDF:  {sdf_out}")

    # 3. Akcja wykonywana po wygenerowaniu SDF
    return [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-world", "default",
                "-file", str(sdf_out),
                "-name", robot_name,
                "-allow_renaming", "false",
            ],
            output="screen",
        )
    ]

def generate_launch_description():
    pkg_velma_moveit_config = get_package_share_directory('velma_moveit_config')
    default_xacro = PathJoinSubstitution([pkg_velma_moveit_config, 'config', 'velma.urdf.xacro'])
    convert_to_sdf_and_spawn = OpaqueFunction(function=generate_sdf_and_spawn)

    return LaunchDescription([
        DeclareLaunchArgument(
            "xacro_file", default_value=default_xacro,
            description="Absolute path to URDF Xacro file.",
        ),
        convert_to_sdf_and_spawn,
    ])

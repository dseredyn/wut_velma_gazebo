# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
import time

import subprocess

from launch.actions import TimerAction

def _try_spawn(context, *args, **kwargs):
    #world = LaunchConfiguration('world_name').perform(context)
    world = 'default'
    # entity = LaunchConfiguration('entity_name').perform(context)
    entity = 'velma'
    # Serwisy Gazebo Transport (nie ROS):
    s1 = f"/world/{world}/create"
    s2 = f"/world/{world}/create_multiple"

    try:
        r = subprocess.run(
            ["gz", "service", "-l"],
            capture_output=True, text=True, timeout=10.0
        )
        print('*** captured output')
        out = r.stdout
    except Exception as e:
        # print(e)
        out = ""

    # print(f'*** {out}')

    if (s1 in out) or (s2 in out):
        print('*** spawn_velma.launch.py: spawning')
        # GOTOWE -> dopiero teraz startujemy Node(create)
        return [Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-world", world,
                "-name", entity,
                "-topic", "robot_description",
                # "-x", LaunchConfiguration('x').perform(context),
                # "-y", LaunchConfiguration('y').perform(context),
                # "-z", LaunchConfiguration('z').perform(context),
                # "-R", LaunchConfiguration('roll').perform(context),
                # "-P", LaunchConfiguration('pitch').perform(context),
                # "-Y", LaunchConfiguration('yaw').perform(context),
            ],
        )]
    print('*** spawn_velma.launch.py: waiting for spawn service - retrying...')

    # Not ready, do not block, try again in 1 second
    return [TimerAction(
        period=1.0,
        actions=[OpaqueFunction(function=_try_spawn)]
    )]



def generate_launch_description():
    # Get the urdf file
    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    # model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    # urdf_path = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'models',
    #     model_folder,
    #     'model.sdf'
    # )

    # Launch configuration variables specific to simulation
    # x_pose = LaunchConfiguration('x_pose', default='0.0')
    # y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    # declare_x_position_cmd = DeclareLaunchArgument(
    #     'x_pose', default_value='0.0',
    #     description='Specify namespace of the robot')

    # declare_y_position_cmd = DeclareLaunchArgument(
    #     'y_pose', default_value='0.0',
    #     description='Specify namespace of the robot')

#ros2 service wait /world/{w}/create_multiple

    # start_gazebo_ros_spawner_cmd = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-name', 'velma',
    #         #'-file', urdf_path,
    #         '-topic', 'robot_description',
    #         # '-x', x_pose,
    #         # '-y', y_pose,
    #         # '-z', '0.01'
    #     ],
    #     output='screen',
    # )

    bridge_params = os.path.join(
        get_package_share_directory('wut_velma_gazebo'),
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
    ld = LaunchDescription()

    # Declare the launch options
    # ld.add_action(declare_x_position_cmd)
    # ld.add_action(declare_y_position_cmd)

    # Add any conditioned actions
    ld.add_action(TimerAction(period=0.1, actions=[OpaqueFunction(function=_try_spawn)]))
    # ld.add_action(OpaqueFunction(function=spawn_when_gz_ready))
    # ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd) # if TURTLEBOT3_MODEL != 'burger' else None

    return ld

#!/usr/bin/env python3
import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess


def _do_convert(context, *args, **kwargs):
    xacro_file = LaunchConfiguration("xacro_file").perform(context)
    xacro_args = LaunchConfiguration("xacro_args").perform(context).strip()
    sdf_out    = LaunchConfiguration("sdf_out").perform(context)
    urdf_out = 'velma.urdf'

    print(f'xacro_file: {xacro_file}')
    cmd = (
        f'xacro {shlex.quote(xacro_file)} {xacro_args} > {shlex.quote(urdf_out)}'
        f' && gz sdf -p {shlex.quote(urdf_out)} > {shlex.quote(sdf_out)}'
        f' && echo "[xacro_to_sdf] Wrote URDF: {urdf_out}"'
        f' && echo "[xacro_to_sdf] Wrote SDF : {sdf_out}"'
        f' && echo "Adding <self_collide>true</self_collide> for the whole model (bug: https://github.com/gazebosim/gz-sim/issues/3261)"'
        f' && sed --follow-symlinks -i "s/<model name=\'velma\'>/<model name=\'velma\'>\\n    <self_collide>true<\\/self_collide>/g" {sdf_out}'
    )

    return [
        ExecuteProcess(
            cmd=["bash", "-lc", cmd],
            output="screen",
        )
    ]


def generate_launch_description():
    pkg_velma_moveit_config = get_package_share_directory('velma_moveit_config')
    pkg_wut_velma_gazebo = get_package_share_directory('wut_velma_gazebo')

    default_xacro = PathJoinSubstitution([pkg_velma_moveit_config, 'config', 'velma.urdf.xacro'])
    default_sdf = PathJoinSubstitution([pkg_wut_velma_gazebo, 'models', 'velma', 'model.sdf'])

    return LaunchDescription([
        DeclareLaunchArgument(
            "xacro_file", default_value=default_xacro,
            description="Absolute path to URDF Xacro file.",
        ),
        DeclareLaunchArgument(
            "xacro_args",
            default_value="collision_detector:=\"dart\"",
            description="xacro parameters, e.g.: 'a:=1 b:=true name:=robot'.",
        ),
        DeclareLaunchArgument(
            "sdf_out",
            default_value=default_sdf,
            description="The result SDF file.",
        ),

        # Conversion xacro -> urdf -> sdf. Do not block other launch actions
        OpaqueFunction(function=_do_convert),
    ])

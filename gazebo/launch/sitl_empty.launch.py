"""
Swarm Digital Twin - SITL Launch (Empty World)
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>

Launches Gazebo with the empty world and spawns an X500 quadrotor
with Ardupilot SITL and the wind perturbation ROS node.

Usage:
    ros2 launch gazebo sitl_empty.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    worlds_dir = os.path.join(pkg_dir, 'worlds')
    models_dir = os.path.join(pkg_dir, 'models')

    world_file = LaunchConfiguration('world')
    wind_speed = LaunchConfiguration('wind_speed')
    wind_dir_x = LaunchConfiguration('wind_dir_x')
    wind_dir_y = LaunchConfiguration('wind_dir_y')
    num_drones = LaunchConfiguration('num_drones')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'world', default_value=os.path.join(worlds_dir, 'empty.world'),
            description='Path to Gazebo world file',
        ),
        DeclareLaunchArgument(
            'wind_speed', default_value='0.0',
            description='Constant wind speed [m/s]',
        ),
        DeclareLaunchArgument(
            'wind_dir_x', default_value='1.0',
            description='Wind direction X component',
        ),
        DeclareLaunchArgument(
            'wind_dir_y', default_value='0.0',
            description='Wind direction Y component',
        ),
        DeclareLaunchArgument(
            'num_drones', default_value='1',
            description='Number of drones to spawn',
        ),

        # Gazebo server
        ExecuteProcess(
            cmd=[
                'gz', 'sim', '-r', '-s', world_file,
            ],
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': models_dir,
            },
        ),

        # Gazebo GUI
        ExecuteProcess(
            cmd=['gz', 'sim', '-g'],
            output='screen',
        ),

        # Spawn X500 model
        ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', '/world/swarm_dt_empty/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req',
                'sdf_filename: "x500/model.sdf", '
                'name: "x500_0", '
                'pose: {position: {x: 0, y: 0, z: 0.15}}',
            ],
            output='screen',
        ),

        # Micro-XRCE-DDS Agent (PX4 <-> ROS 2 bridge)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
        ),

        # Wind perturbation node (altitude-dependent ISA density, position-aware)
        Node(
            package='gazebo',
            executable='wind_node.py',
            name='wind_node',
            parameters=[{
                'wind_speed': wind_speed,
                'wind_direction_x': wind_dir_x,
                'wind_direction_y': wind_dir_y,
                'turbulence_type': 'constant',
                'base_altitude_msl': 0.0,  # override for altitude-specific sites
            }],
            output='screen',
        ),
    ])

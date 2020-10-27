#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    pkg_two_wheel_gazebo = get_package_share_directory('two_wheel_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    urdf_file_name = 'two_wheel.gazebo'

    world_file_name = 'two_wheel.world'

    urdf = os.path.join(
        pkg_two_wheel_gazebo,
        urdf_file_name)

    world = os.path.join(
        pkg_two_wheel_gazebo,
        world_file_name)

    return LaunchDescription([
         DeclareLaunchArgument(
          'world',
          default_value=[world, ''],
          description='SDF world file'),
          gazebo,
         Node(
             package='gazebo_ros',
             node_executable='spawn_entity.py',
             arguments=['-entity', 'two_wheel', '-file', urdf],
             output='screen'),
    ])
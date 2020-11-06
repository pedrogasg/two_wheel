#!/usr/bin/env python3
import os
import xacro
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    pkg_dir = get_package_share_directory('two_wheel_simulation')

    xacro_file_name = 'two_wheel.xacro'

    world_file_name = 'two_wheel.world'

    world = os.path.join(
        pkg_dir, 'world',
        world_file_name)

    xacro_model = os.path.join(
        pkg_dir, 'urdf',
        xacro_file_name)

    # create temp file 
    urdf = tempfile.mktemp(prefix='%s_' % os.path.basename(xacro_model))

    # open and process file
    doc = xacro.process_file(xacro_model)
    # open the output file
    out = xacro.open_output(urdf)

    robot_desc = doc.toprettyxml(indent='  ')
    
    # write description
    out.write(robot_desc)

    return LaunchDescription([
         DeclareLaunchArgument(
          'world',
          default_value=[world, ''],
          description='SDF world file'),
          gazebo,
         Node(
             package='gazebo_ros',
             executable='spawn_entity.py',
             arguments=['-entity', 'two_wheel', '-file', urdf],
             output='screen'),
    ])
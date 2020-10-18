#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    urdf_file_name = 'two_wheel.urdf.xml'

    config_file_name = 'urdf.rviz'

    rviz_config = os.path.join(
        get_package_share_directory('two_wheel_urdf'),
        config_file_name)

    urdf = os.path.join(
        get_package_share_directory('two_wheel_urdf'),
        urdf_file_name)

    return LaunchDescription([
        
        Node(
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            arguments=[urdf]),

        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]),

        Node(
            package='rviz2',
            node_executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]),
    ])
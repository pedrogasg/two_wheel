#!/usr/bin/env python3
import os
import xacro
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    config_file = LaunchConfiguration('config_file')
    robot_description = LaunchConfiguration('robot_description')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_dir = get_package_share_directory('two_wheel_simulation')

    xacro_file_name = 'two_wheel.xacro'

    rviz_file_name = 'urdf.rviz'

    rviz_config = os.path.join(
        pkg_dir, 'rviz',
        rviz_file_name)

    xacro_model = os.path.join(
        pkg_dir, 'urdf',
        xacro_file_name)

    # create temp file 
    # urdf = tempfile.mktemp(prefix='%s_' % os.path.basename(xacro_model))

    # open and process file
    doc = xacro.process_file(xacro_model)
    # open the output file
    # out = xacro.open_output(urdf)

    robot_desc = doc.toprettyxml(indent='  ')
    
    # write description
    # out.write(robot_desc)

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_description': robot_description}

    configured_params = RewrittenYaml(
        source_file=config_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)
    print(configured_params)
    return LaunchDescription([
        DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'two_wheel.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes.'),

        DeclareLaunchArgument(
        'robot_description',
        default_value=robot_desc,
        description='Full robot description'),

        DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[configured_params]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            remappings=[('/joint_states', '/joint_states')],
            parameters=[configured_params]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]),
    ])
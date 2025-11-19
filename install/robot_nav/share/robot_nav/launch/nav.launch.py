#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # Package share for RViz config
    pkg_share = FindPackageShare(package='robot_nav').find('robot_nav')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # Launch configurations (arguments)
    rvizconfig   = LaunchConfiguration('rvizconfig')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # RViz node (no robot_description, just your config and clock)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # robot_nav nodes
    data_node = Node(
        package='robot_nav',
        executable='data_node',
        name='data_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lpf_node = Node(
        package='robot_nav',
        executable='lpf_node',
        name='lpf_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bias_correct_node = Node(
        package='robot_nav',
        executable='bias_correct_node',
        name='bias_correct_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    complementary_position_node = Node(
        package='robot_nav',
        executable='complementary_position_node',
        name='complementary_position_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to RViz config file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Nodes
        rviz_node,
        data_node,
        lpf_node,
        bias_correct_node,
        complementary_position_node,
    ])

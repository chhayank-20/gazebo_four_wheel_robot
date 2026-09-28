#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='/workspace/gazebo_four_wheel_robot/config/nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator'
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=remappings
    )

    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=remappings
    )

    behavior_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=remappings
    )

    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        remappings=remappings
    )

    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(controller_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(behavior_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld

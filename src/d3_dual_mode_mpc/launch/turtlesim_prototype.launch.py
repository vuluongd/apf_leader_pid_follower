"""
Launch file for D3 Dual-Mode MPC — Turtlesim 2D Prototype.

Launches:
  1. turtlesim_node (simulator)
  2. Spawn obstacle turtles
  3. apf_leader (APF planner for turtle1)
  4. signal_simulator (RhOct emulator)
  5. mpc_follower (Dual-Mode MPC for turtle2)
  6. velocity_relay (RF broadcast, Case B)
  7. data_logger (CSV logging)
"""

import os
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, DeclareLaunchArgument,
                            TimerAction, LogInfo)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('d3_dual_mode_mpc')
    config_file = os.path.join(pkg_share, 'config', 'default_params.yaml')

    # Launch arguments
    controller_mode_arg = DeclareLaunchArgument(
        'controller_mode', default_value='mpc_b',
        description='Controller: hover, mpc_a, mpc_b, mpc_visual'
    )
    delay_arg = DeclareLaunchArgument(
        'delay_h', default_value='0.25',
        description='Sensing delay in seconds'
    )
    k_rep_arg = DeclareLaunchArgument(
        'k_rep', default_value='2.0',
        description='APF repulsive gain'
    )

    # Turtlesim
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    # Spawn follower turtle (turtle2) at offset position
    spawn_follower = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn',
             'turtlesim/srv/Spawn',
             '{x: 5.5, y: 3.5, theta: 0.0, name: "turtle2"}'],
        output='screen',
    )

    # Spawn obstacle turtles
    spawn_obstacle1 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn',
             'turtlesim/srv/Spawn',
             '{x: 7.0, y: 5.5, theta: 0.0, name: "obstacle1"}'],
        output='screen',
    )
    spawn_obstacle2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn',
             'turtlesim/srv/Spawn',
             '{x: 4.0, y: 8.0, theta: 0.0, name: "obstacle2"}'],
        output='screen',
    )

    # APF Leader
    apf_leader = Node(
        package='d3_dual_mode_mpc',
        executable='apf_leader',
        name='apf_leader',
        output='screen',
        parameters=[config_file, {
            'k_rep': LaunchConfiguration('k_rep'),
        }],
    )

    # Signal Simulator
    signal_sim = Node(
        package='d3_dual_mode_mpc',
        executable='signal_simulator',
        name='signal_simulator',
        output='screen',
        parameters=[config_file, {
            'delay_h': LaunchConfiguration('delay_h'),
        }],
    )

    # MPC Follower
    mpc_follower = Node(
        package='d3_dual_mode_mpc',
        executable='mpc_follower',
        name='mpc_follower',
        output='screen',
        parameters=[config_file, {
            'controller_mode': LaunchConfiguration('controller_mode'),
        }],
    )

    # Velocity Relay (for Case B)
    vel_relay = Node(
        package='d3_dual_mode_mpc',
        executable='velocity_relay',
        name='velocity_relay',
        output='screen',
        parameters=[config_file],
    )

    # Data Logger
    data_logger = Node(
        package='d3_dual_mode_mpc',
        executable='data_logger',
        name='data_logger',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        controller_mode_arg,
        delay_arg,
        k_rep_arg,

        LogInfo(msg='Starting D3 Dual-Mode MPC Turtlesim Prototype...'),

        turtlesim_node,

        # Delay spawns to let turtlesim start
        TimerAction(period=2.0, actions=[
            spawn_follower,
            spawn_obstacle1,
            spawn_obstacle2,
        ]),

        # Delay control nodes to let turtles spawn
        TimerAction(period=4.0, actions=[
            apf_leader,
            signal_sim,
            mpc_follower,
            vel_relay,
            data_logger,
        ]),
    ])

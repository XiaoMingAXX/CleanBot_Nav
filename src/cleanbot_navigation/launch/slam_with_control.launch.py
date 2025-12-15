#!/usr/bin/env python3
"""
完整的SLAM建图启动文件
包含：机器人控制、雷达、SLAM
适合独立运行建图测试
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    nav_pkg_dir = get_package_share_directory('cleanbot_navigation')
    cleanbot_control_dir = get_package_share_directory('cleanbot_control')
    rplidar_pkg_dir = get_package_share_directory('rplidar_ros')
    
    # 配置文件
    slam_config = os.path.join(nav_pkg_dir, 'config', 'slam_toolbox.yaml')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock')
    
    # 1. 启动机器人控制系统（包括TF、里程计等）
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cleanbot_control_dir, 'launch', 'robot_bringup.launch.py')
        )
    )
    
    # 2. 启动激光雷达
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg_dir, 'launch', 'rplidar_a1_launch.py')
        )
    )
    
    # 3. 启动SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 4. 生命周期管理器（自动激活SLAM节点）
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['slam_toolbox']}
        ]
    )
    
    # 延迟启动SLAM，等待所有TF就绪
    slam_node_with_delay = TimerAction(
        period=5.0,  # 等待机器人控制和雷达都启动完成
        actions=[slam_node, lifecycle_manager]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(robot_bringup)     # 机器人控制系统
    ld.add_action(rplidar_launch)    # 雷达
    ld.add_action(slam_node_with_delay)  # SLAM（延迟启动）
    
    return ld


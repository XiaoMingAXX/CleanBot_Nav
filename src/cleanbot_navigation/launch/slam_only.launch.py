#!/usr/bin/env python3
"""
仅启动SLAM建图的启动文件
用于快速测试建图功能
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # 获取包路径
    nav_pkg_dir = get_package_share_directory('cleanbot_navigation')
    rplidar_pkg_dir = get_package_share_directory('rplidar_ros')
    
    # 配置文件
    slam_config = os.path.join(nav_pkg_dir, 'config', 'slam_toolbox.yaml')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock')
    
    # 启动激光雷达
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg_dir, 'launch', 'rplidar_a1_launch.py')
        )
    )
    
    # 启动SLAM Toolbox
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
    
    # 生命周期管理器（自动激活SLAM节点）
    # 注意：如果slam_toolbox.yaml中use_lifecycle_manager为false，此节点不是必需的
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
    
    # 用定时器延迟启动SLAM节点，确保雷达驱动已上线
    slam_node_with_delay = TimerAction(
        period=3.0,  # 等待雷达启动
        actions=[slam_node, lifecycle_manager]  # 同时启动SLAM和管理器
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(rplidar_launch)
    ld.add_action(slam_node_with_delay)  # 延迟启动SLAM
    
    return ld



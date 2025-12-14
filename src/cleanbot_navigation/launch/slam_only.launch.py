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
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(rplidar_launch)
    ld.add_action(slam_node)
    
    return ld


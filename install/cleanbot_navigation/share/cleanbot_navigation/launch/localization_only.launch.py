#!/usr/bin/env python3
"""
仅启动定位的启动文件
用于在已有地图上进行定位测试
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
    amcl_config = os.path.join(nav_pkg_dir, 'config', 'amcl.yaml')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock')
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_pkg_dir, 'maps', 'default_map.yaml'),
        description='Full path to map yaml file')
    
    # 启动激光雷达
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg_dir, 'launch', 'rplidar_a1_launch.py')
        )
    )
    
    # 启动Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 启动AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config]
    )
    
    # 生命周期管理器（用于激活map_server和amcl）
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)
    ld.add_action(rplidar_launch)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager)
    
    return ld



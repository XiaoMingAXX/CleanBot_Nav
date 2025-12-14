#!/usr/bin/env python3
"""
导航系统启动文件
启动所有导航相关节点：
- 激光雷达
- SLAM工具箱
- AMCL定位
- Nav2导航栈
- 模式管理器
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取包路径
    nav_pkg_dir = get_package_share_directory('cleanbot_navigation')
    rplidar_pkg_dir = get_package_share_directory('rplidar_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 配置文件路径
    slam_config = os.path.join(nav_pkg_dir, 'config', 'slam_toolbox.yaml')
    nav2_config = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')
    amcl_config = os.path.join(nav_pkg_dir, 'config', 'amcl.yaml')
    
    # 地图目录
    map_dir = os.path.join(nav_pkg_dir, 'maps')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load (for navigation mode)')
    
    # 1. 启动激光雷达
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg_dir, 'launch', 'rplidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser',
        }.items()
    )
    
    # 2. 启动SLAM Toolbox（异步模式，生命周期管理）
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ],
        # 使用生命周期节点，初始状态为unconfigured
        # 模式管理器会控制其激活
    )
    
    # 3. 启动AMCL（生命周期管理）
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config],
    )
    
    # 4. 启动Map Server（生命周期管理，用于导航模式）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # 5. 启动Nav2生命周期管理器
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]}
        ]
    )
    
    # 6. 启动Nav2导航栈
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_config,
        }.items()
    )
    
    # 7. 启动导航模式管理器
    mode_manager_node = Node(
        package='cleanbot_navigation',
        executable='navigation_mode_manager',
        name='navigation_mode_manager',
        output='screen',
        parameters=[
            {'map_save_dir': os.path.expanduser('~/cleanbot_maps')},
            {'default_map_name': 'cleanbot_map'}
        ]
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_map)
    
    # 添加节点
    ld.add_action(rplidar_launch)
    ld.add_action(slam_toolbox_node)
    ld.add_action(amcl_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_nav)
    ld.add_action(nav2_bringup)
    ld.add_action(mode_manager_node)
    
    return ld


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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml



def generate_launch_description():
       # 基础环境变量设置
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')
    set_localhost = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', 'false')
    # 获取包路径
    nav_pkg_dir = get_package_share_directory('cleanbot_navigation')
    rplidar_pkg_dir = get_package_share_directory('rplidar_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 获取源码目录路径（用于地图保存，不随编译改变）
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    package_src_dir = os.path.dirname(launch_file_dir)  # 从 launch 目录到软件包根目录
    #map_dir = os.path.join(package_src_dir, 'maps')
    # 正确解析~为用户主目录
    map_dir = os.path.expanduser('~/cleanbot_maps')
    # 可选：确保目录存在（避免保存地图时因目录不存在报错）
    os.makedirs(map_dir, exist_ok=True)  # exist_ok=True：目录已存在时不报错
    
    # 配置文件路径
    slam_config = os.path.join(nav_pkg_dir, 'config', 'slam_toolbox.yaml')
    nav2_config = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')
    amcl_config = os.path.join(nav_pkg_dir, 'config', 'amcl.yaml')
    
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
        default_value='false',
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
        parameters=[
            amcl_config,
            {'use_sim_time': use_sim_time},
            {'transform_tolerance': 0.5},  # 允许0.5s的时间差（足够覆盖仿真中的延迟）
            ],
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
            {'map_save_dir': map_dir},
            {'default_map_name': 'cleanbot_map'}
        ]
    )
    
    # 8. 启动RViz2
    rviz_config = os.path.join(nav_pkg_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',  # 仍输出到屏幕，但仅输出ERROR级
        ros_arguments=['--log-level', 'ERROR']  # 只打印ERROR及以上日志
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    ld.add_action(set_domain_id)
    ld.add_action(set_localhost)
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
    #ld.add_action(rviz_node)
    
    return ld



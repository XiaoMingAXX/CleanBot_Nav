#!/usr/bin/env python3
"""
导航系统仿真启动文件（适配Gazebo）
启动所有导航相关节点（仿真版）：
- SLAM工具箱（适配仿真激光）
- AMCL定位（优化仿真时间同步）
- Nav2导航栈（使用sim配置文件）
- 模式管理器
- 移除真实激光雷达依赖（使用Gazebo发布的/scan）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction,
    SetEnvironmentVariable, LogInfo
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ====================== 基础配置（仿真专属）======================
    # 基础环境变量设置（替换弃用的ROS_LOCALHOST_ONLY）
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')
    set_discovery_range = SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCAL')  # 新环境变量

    # 获取包路径
    nav_pkg_dir = get_package_share_directory('cleanbot_navigation')
    nav2_bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')  # 新增：BT导航器包路径

    # 获取源码目录路径（用于地图保存，不随编译改变）
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    package_src_dir = os.path.dirname(launch_file_dir)  # 从 launch 目录到软件包根目录
    # 正确解析~为用户主目录（仿真环境同样适用）
    map_dir = os.path.expanduser('~/cleanbot_maps')
    # 确保目录存在（避免保存地图时报错）
    os.makedirs(map_dir, exist_ok=True)

    # ====================== 配置文件路径（仿真专属）======================
    # 替换为仿真专用的Nav2配置文件
    slam_config = os.path.join(nav_pkg_dir, 'config', 'slam_toolbox.yaml')
    nav2_config = os.path.join(nav_pkg_dir, 'config', 'nav2_sim_params.yaml')  # 仿真配置文件
    amcl_config = os.path.join(nav_pkg_dir, 'config', 'amcl.yaml')

    # 关键：重写Nav2参数，解析BT XML为绝对路径
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_config,
        root_key='',
        param_rewrites={
            'bt_navigator.default_nav_to_pose_bt_xml': os.path.join(nav2_bt_navigator_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
            'bt_navigator.default_nav_through_poses_bt_xml': os.path.join(nav2_bt_navigator_dir, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml')
        },
        convert_types=True
    )

    # ====================== 启动参数（仿真专属）======================
    # 仿真环境默认启用use_sim_time，autostart默认开启
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # 仿真环境强制启用仿真时间
        description='Use simulation (Gazebo) clock if true')

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='false',  # 不自动启动，由mode_manager控制
        description='Automatically startup the nav2 stack')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load (for navigation mode)')

    # ====================== 节点定义（仿真专属调整）======================
    # 1. 【移除真实激光雷达】仿真环境下激光由Gazebo插件发布，无需启动rplidar
    remap_scan_topic = SetRemap(
        src='/scan',  # Gazebo发布的激光话题
        dst='/scan'   # 导航栈订阅的激光话题（保持一致，可根据需要修改）
    )

    # 2. 启动SLAM Toolbox（异步模式，适配仿真时间）
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time},
            {'tf_cache_time': 10.0},
            {'transform_timeout': 0.5}
        ],
        remappings=[
            ('/scan', '/scan')
        ]
    )

    # 3. 启动AMCL（生命周期管理，仿真时间优化）
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': use_sim_time},
            {'transform_tolerance': 0.5},
            {'tf_buffer_duration': 10.0}  # 新增：TF缓存时长
        ],
        remappings=[
        ('/odom', '/odometry/filtered')  # AMCL订阅你的里程计话题
        ]
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
    
    # 5. Nav2核心节点（使用重写后的参数）
    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
        ('/cmd_vel', '/diff_drive_controller/cmd_vel'),  # 速度指令发布到你的底盘话题
        ('/odom', '/odometry/filtered')  # 订阅你的里程计话题（双重保障）
    ]
    )
    
    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}],
    )
    
    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}],
    )
    
    # BT Navigator（使用重写后的参数，解决路径问题）
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}],
    )
    
    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}],
    )
    
    # Smoother Server（替代velocity_smoother，Jazzy版本）
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}],
    )

    # 6. 启动Nav2生命周期管理器（修正节点列表）
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
                'smoother_server'  # 替换velocity_smoother
            ]}
        ]
    )
    
    # 7. 启动导航模式管理器（适配仿真环境）
    mode_manager_node = Node(
        package='cleanbot_navigation',
        executable='navigation_mode_manager',
        name='navigation_mode_manager',
        output='screen',
        parameters=[
            {'map_save_dir': map_dir},
            {'default_map_name': 'cleanbot_map'},
            {'use_sim_time': use_sim_time}
        ]
    )

    # # 8. 启动RViz2（仿真环境专属配置）
    # rviz_config = os.path.join(nav_pkg_dir, 'rviz', 'nav2_sim_view.rviz')
    # if not os.path.exists(rviz_config):
    #     rviz_config = os.path.join(nav_pkg_dir, 'rviz', 'nav2_default_view.rviz')

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    #     ros_arguments=['--log-level', 'ERROR']
    # )
# 8. 启动RViz2（改为获取 Nav2 官方配置）
    # 获取 nav2_bringup 包的路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 官方默认配置文件路径
    rviz_config_path = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path], # 使用官方路径
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        ros_arguments=['--log-level', 'ERROR']
    )
    # ====================== 组装启动描述 ======================
    ld = LaunchDescription()
    # 添加环境变量（替换弃用的）
    ld.add_action(set_domain_id)
    ld.add_action(set_discovery_range)
    # 添加参数声明
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_map)
    # 添加激光话题重映射
    ld.add_action(remap_scan_topic)

    # 添加核心节点
    ld.add_action(slam_toolbox_node)
    ld.add_action(amcl_node)
    ld.add_action(map_server_node)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(smoother_server)  # 新增smoother_server
    ld.add_action(lifecycle_manager_nav)
    ld.add_action(mode_manager_node)
    ld.add_action(rviz_node)

    # 打印提示
    ld.add_action(LogInfo(msg='========== CleanBot 仿真导航启动成功 =========='))
    ld.add_action(LogInfo(msg=f'地图保存路径: {map_dir}'))
    ld.add_action(LogInfo(msg=f'Nav2配置文件: {nav2_config}'))

    return ld

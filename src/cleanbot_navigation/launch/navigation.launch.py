#!/usr/bin/env python3
"""
CleanBot 统一导航启动文件
支持实机和仿真两种模式，通过 sim_mode 参数切换：
- sim_mode=true: 仿真模式（使用Gazebo激光，启用仿真时间，启动RViz）
- sim_mode=false: 实机模式（使用真实激光雷达，关闭仿真时间）

启动方法：
- 仿真：ros2 launch cleanbot_navigation navigation.launch.py sim_mode:=true
- 实机：ros2 launch cleanbot_navigation navigation.launch.py sim_mode:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, 
    GroupAction, SetEnvironmentVariable, LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # ====================== 启动参数声明 ======================
    sim_mode = LaunchConfiguration('sim_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    enable_rviz = LaunchConfiguration('enable_rviz')
    
    declare_sim_mode = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='是否为仿真模式 (true=仿真, false=实机)')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value=PythonExpression(["'", sim_mode, "'"]),  # 自动根据sim_mode设置
        description='是否使用仿真时间 (Gazebo clock)')
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='是否自动启动Nav2栈 (由mode_manager控制，建议false)')
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='地图YAML文件完整路径 (导航模式使用)')
    
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value=PythonExpression(["'", sim_mode, "'"]),  # 仿真模式默认启动RViz
        description='是否启动RViz可视化')
    
    # ====================== 基础配置 ======================
    # 环境变量设置（根据模式自动调整）
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')
    # 仿真模式用LOCALHOST，实机用SUBNET（条件设置需要用PythonExpression）
    set_discovery_range = SetEnvironmentVariable(
        'ROS_AUTOMATIC_DISCOVERY_RANGE',
        PythonExpression(['"LOCALHOST" if "', sim_mode, '" == "true" else "SUBNET"'])
    )
    
    # 获取包路径
    nav_pkg_dir = get_package_share_directory('cleanbot_navigation')
    nav2_bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')
    
    # 地图保存目录
    map_dir = os.path.expanduser('~/cleanbot_maps')
    os.makedirs(map_dir, exist_ok=True)
    
    # ====================== 配置文件路径 ======================
    slam_config = os.path.join(nav_pkg_dir, 'config', 'slam_toolbox.yaml')
    nav2_config = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')  # 统一使用同一配置
    amcl_config = os.path.join(nav_pkg_dir, 'config', 'amcl.yaml')
    cleaning_task_config = os.path.join(nav_pkg_dir, 'config', 'cleaning_task_params.yaml')
    
    # 重写Nav2参数，解析BT XML为绝对路径
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_config,
        root_key='',
        param_rewrites={
            'bt_navigator.default_nav_to_pose_bt_xml': os.path.join(
                nav2_bt_navigator_dir, 'behavior_trees', 
                'navigate_to_pose_w_replanning_and_recovery.xml'),
            'bt_navigator.default_nav_through_poses_bt_xml': os.path.join(
                nav2_bt_navigator_dir, 'behavior_trees', 
                'navigate_through_poses_w_replanning_and_recovery.xml')
        },
        convert_types=True
    )
    
    # ====================== 条件节点：激光雷达 ======================
    # 实机模式：启动真实激光雷达
    rplidar_pkg_dir = get_package_share_directory('rplidar_ros')
    rplidar_launch = GroupAction(
        condition=UnlessCondition(sim_mode),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rplidar_pkg_dir, 'launch', 'rplidar_a1_launch.py')
                ),
                launch_arguments={
                    'serial_port': '/dev/radar_serial',
                    'frame_id': 'laser',
                }.items()
            )
        ]
    )
    
    # 仿真模式：不启动激光雷达（Gazebo自带）
    # 无需额外节点，只需确保话题映射正确
    
    # ====================== 核心节点定义 ======================
    # 1. SLAM Toolbox（实机和仿真共用，参数自动适配）
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time},
            {'tf_cache_time': 2.0},
            # 仿真模式容差更大，实机更小
            {'transform_timeout': PythonExpression([
                '0.5 if "', sim_mode, '" == "true" else 0.3'
            ])}
        ],
        remappings=[('/scan', '/scan')]
    )
    
    # 2. AMCL定位
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': use_sim_time},
            # 仿真模式容差更大
            {'transform_tolerance': PythonExpression([
                '0.5 if "', sim_mode, '" == "true" else 0.05'
            ])},
            {'tf_buffer_duration': PythonExpression([
                '5.0 if "', sim_mode, '" == "true" else 2.0'
            ])}
        ],
        remappings=[('/odom', '/odometry/filtered')]
    )
    
    # 3. Map Server
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
    
    # 4. Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel'),
            ('/odom', '/odometry/filtered')
        ]
    )
    
    # 5. Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}]
    )
    
    # 6. Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}]
    )
    
    # 7. BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}]
    )
    
    # 8. Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}]
    )
    
    # 9. Smoother Server
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': use_sim_time}]
    )
    
    # 10. Nav2生命周期管理器
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
                'smoother_server'
            ]}
        ]
    )
    
    # 11. 导航模式管理器
    mode_manager_node = Node(
        package='cleanbot_navigation',
        executable='navigation_mode_manager.py',
        name='navigation_mode_manager',
        output='screen',
        parameters=[
            {'map_save_dir': map_dir},
            {'default_map_name': 'cleanbot_map'},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 12. 清扫任务管理节点
    cleaning_task_node = Node(
        package='cleanbot_navigation',
        executable='cleaning_task_node.py',
        name='cleaning_task',
        output='screen',
        parameters=[
            cleaning_task_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 13. RViz2（条件启动）
    rviz_config_path = os.path.join(nav_pkg_dir, 'rviz', 'cleanbot_nav_view.rviz')
    rviz_node = Node(
        condition=IfCondition(enable_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        ros_arguments=['--log-level', 'ERROR']
    )
    
    # ====================== 组装启动描述 ======================
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_sim_mode)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_map)
    ld.add_action(declare_enable_rviz)
    
    # 添加环境变量
    ld.add_action(set_domain_id)
    ld.add_action(set_discovery_range)
    
    # 添加条件节点（激光雷达）
    ld.add_action(rplidar_launch)  # 只在实机模式启动
    
    # 添加核心节点
    ld.add_action(slam_toolbox_node)
    ld.add_action(amcl_node)
    ld.add_action(map_server_node)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(smoother_server)
    ld.add_action(lifecycle_manager_nav)
    ld.add_action(mode_manager_node)
    ld.add_action(cleaning_task_node)
    ld.add_action(rviz_node)  # 条件启动
    
    # 打印提示
    ld.add_action(LogInfo(msg=PythonExpression([
        '"========== CleanBot 仿真导航启动成功 ==========" if "', sim_mode, 
        '" == "true" else "========== CleanBot 实机导航启动成功 =========="'
    ])))
    ld.add_action(LogInfo(msg=f'地图保存路径: {map_dir}'))
    ld.add_action(LogInfo(msg=f'Nav2配置文件: {nav2_config}'))
    ld.add_action(LogInfo(msg=PythonExpression([
        '"模式: 仿真 (Gazebo)" if "', sim_mode, 
        '" == "true" else "模式: 实机 (真实硬件)"'
    ])))
    
    return ld


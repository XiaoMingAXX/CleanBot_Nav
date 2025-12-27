#!/usr/bin/env python3
"""
CleanBot Gazebo仿真启动文件 (Gazebo Harmonic / gz-sim版本)
功能：
1. 启动Gazebo仿真环境
2. 加载机器人模型
3. 启动ros2_control控制器
4. 启动Web控制界面
5. 启动EKF定位
6. 启动导航系统
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    TimerAction, 
    SetEnvironmentVariable,
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution, 
    Command
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 环境变量设置
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')
    set_localhost = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', 'false')
    
    # 设置Gazebo插件路径
    set_gz_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        '/opt/ros/jazzy/lib'
    )
    
    # Launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')
    web_port = LaunchConfiguration('web_port', default='8080')
    
    # 包路径
    pkg_cleanbot_control = get_package_share_directory('cleanbot_control')
    pkg_cleanbot_navigation = get_package_share_directory('cleanbot_navigation')
    
    # 配置文件路径
    controllers_config = PathJoinSubstitution([
        pkg_cleanbot_control, 'config', 'cleanbot_controllers.yaml'
    ])
    
    ekf_config = PathJoinSubstitution([
        pkg_cleanbot_control, 'config', 'ekf.yaml'
    ])
    
    manual_control_config = PathJoinSubstitution([
        pkg_cleanbot_control, 'config', 'manual_control_params.yaml'
    ])
    
    voice_control_config = PathJoinSubstitution([
        pkg_cleanbot_control, 'config', 'voice_control_params.yaml'
    ])
    
    # 使用Gazebo仿真的URDF
    urdf_file = PathJoinSubstitution([
        pkg_cleanbot_control, 'config', 'cleanbot_gazebo.urdf.xacro'
    ])
    
    # 处理xacro文件生成robot_description
    robot_description = Command(['xacro ', urdf_file])
    
    # ==================== 世界文件 ====================
    world_file = PathJoinSubstitution([
        pkg_cleanbot_control, 'world', 'simple_room.sdf'
    ])
    
    # ==================== 节点定义 ====================
    
    # 1. 启动新版Gazebo (gz sim)
    world_file = PathJoinSubstitution([
        pkg_cleanbot_control, 'world', 'simple_room.sdf'
    ])
    
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # 2. 桥接时钟话题
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # 3. 桥接Gazebo话题到ROS2 (IMU)
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu_raw@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen',
        remappings=[
            ('/imu_raw', 'imu/data_raw'),
        ]
    )
    
    # 4. 桥接Gazebo话题到ROS2 (LaserScan) - 保持原始话题名
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan_raw@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # 5. LaserScan frame_id 修复节点 - 修改frame_id并重新发布
    scan_frame_relay = Node(
        package='cleanbot_control',
        executable='laser_frame_relay.py',
        name='laser_frame_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # 6. 在Gazebo中生成机器人 (延迟3秒等待Gazebo启动)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'cleanbot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1',
                ],
                output='screen'
            )
        ]
    )
    
    # 7. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }]
    )
    
    # 8. Web控制节点
    web_control_node = Node(
        package='cleanbot_control',
        executable='web_control_node.py',
        name='web_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'web_port': web_port
        }]
    )
    
    # 8.5. 手动控制节点
    manual_control_node = Node(
        package='cleanbot_control',
        executable='manual_control_node.py',
        name='manual_control_node',
        output='screen',
        parameters=[
            manual_control_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 8.6. 语音控制节点
    voice_control_node = Node(
        package='cleanbot_control',
        executable='voice_control_node.py',
        name='voice_control_node',
        output='screen',
        parameters=[
            voice_control_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 9. 差速控制器spawner - 延迟8秒启动（等待Gazebo的controller_manager启动）
    diff_drive_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller'],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # 10. 关节状态广播器spawner - 延迟8秒启动
    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # 11. EKF定位节点 - 延迟10秒启动
    ekf_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_config, 
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )
    
    # 12. 导航系统 - 延迟13秒启动，确保EKF定位稳定
    navigation_launch = TimerAction(
        period=13.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        pkg_cleanbot_navigation, 'launch', 'navigation.launch.py'
                    ])
                ),
                launch_arguments={
                    'sim_mode': 'true',  # 仿真模式
                    'use_sim_time': use_sim_time,
                    'autostart': 'false',  # 由mode_manager控制
                    'enable_rviz': 'true',  # 仿真默认启动RViz
                }.items()
            )
        ]
    )
    
    # ==================== Launch描述 ====================
    
    return LaunchDescription([
        # 环境变量
        set_domain_id,
        set_localhost,
        set_gz_plugin_path,
        
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo headless (no GUI)'
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Web control panel port'
        ),
        
        # 启动所有节点（按顺序）
        gz_sim,                     # 0秒：启动新版Gazebo（内含controller_manager）
        bridge_clock,               # 0秒：桥接时钟话题（重要！）
        bridge_imu,                 # 0秒：桥接IMU数据
        bridge_scan,                # 0秒：桥接激光雷达数据
        scan_frame_relay,           # 0秒：修改frame_id并重新发布
        robot_state_publisher,      # 0秒：发布robot_description和TF树
        spawn_robot,                # 3秒：在Gazebo中生成机器人
        web_control_node,           # 0秒：启动Web控制
        manual_control_node,        # 0秒：启动手动控制节点
        voice_control_node,         # 0秒：启动语音控制节点
        diff_drive_spawner,         # 8秒：等待Gazebo的controller_manager
        joint_state_broadcaster_spawner,  # 8秒：等待controller_manager
        ekf_node,                   # 10秒：等待TF树完整
        navigation_launch,          # 13秒：启动导航系统（等待EKF稳定）
    ])


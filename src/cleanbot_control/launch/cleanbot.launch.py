#!/usr/bin/env python3
"""
CleanBot完整系统启动文件
包含所有功能：USB通讯、Web控制、ros2_control、EKF定位
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


  
def generate_launch_description():
    # 设置环境变量（避免 ROS2 网络冲突）
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')
    
    # 声明launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    port = LaunchConfiguration('port', default='/dev/ttyACM0')
    baudrate = LaunchConfiguration('baudrate', default='921600')
    web_port = LaunchConfiguration('web_port', default='8080')
    
    # 获取配置文件路径
    pkg_cleanbot_control = get_package_share_directory('cleanbot_control')
    
    controllers_config = PathJoinSubstitution([
        pkg_cleanbot_control, 'config', 'cleanbot_controllers.yaml'
    ])
    
    ekf_config = PathJoinSubstitution([
        pkg_cleanbot_control, 'config', 'ekf.yaml'
    ])
    
    # 使用真实硬件的URDF（包含ros2_control标签）
    urdf_file = PathJoinSubstitution([
        pkg_cleanbot_control, 'config', 'cleanbot_real.urdf.xacro'
    ])
    
    # 处理xacro文件生成robot_description
    robot_description = Command(['xacro ', urdf_file])
    
    # ==================== 节点定义 ====================
    
    # 1. Robot State Publisher - 发布robot_description和TF树
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
    
    # 2. USB通讯节点
    usb_comm_node = Node(
        package='cleanbot_control',
        executable='usb_communication_node.py',
        name='usb_communication_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': port,
            'baudrate': baudrate,
            'auto_reconnect': True,
            'reconnect_interval': 2.0,
            'control_rate': 50.0
        }]
    )
    
    # 3. Web控制节点
    # 发布 TwistStamped 到 cmd_vel，重映射到 /diff_drive_controller/cmd_vel
    # 控制器配置 use_stamped_vel: true，监听 /diff_drive_controller/cmd_vel
    web_control_node = Node(
        package='cleanbot_control',
        executable='web_control_node.py',
        name='web_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'web_port': web_port
        }],
        remappings=[
            ('cmd_vel', '/diff_drive_controller/cmd_vel'),
        ]
    )
    
    # 4. Controller Manager (ros2_control) - 延迟2秒启动
    controller_manager = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[controllers_config, {'robot_description': robot_description}],
                output='screen'
            )
        ]
    )
    
    # 5. 差速控制器spawner - 延迟4秒启动
    diff_drive_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller'],
                output='screen'
            )
        ]
    )
    
    # 6. 关节状态广播器spawner - 延迟4秒启动
    joint_state_broadcaster_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )
    
    # 7. EKF定位节点 - 延迟5秒启动，确保TF树完整
    ekf_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config, {'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # ==================== Launch描述 ====================
    
    return LaunchDescription([
        # 设置环境变量（避免 ROS2 网络冲突）
        set_domain_id,
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='USB serial port'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='921600',
            description='Serial port baudrate'
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Web control panel port'
        ),
        
        # 启动所有节点（按顺序）
        robot_state_publisher,      # 0秒：立即启动，发布URDF和TF
        usb_comm_node,              # 0秒：立即启动
        web_control_node,           # 0秒：立即启动
        controller_manager,         # 2秒：等待robot_description
        diff_drive_spawner,         # 4秒：等待controller_manager
        joint_state_broadcaster_spawner,  # 4秒：等待controller_manager
        ekf_node,                   # 5秒：等待TF树完整
    ])


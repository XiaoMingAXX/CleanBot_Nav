#!/usr/bin/env python3
"""
远程RViz启动文件（主机端）
用于在主机PC上监控和控制树莓派上运行的导航系统（ROS2版本）
注意：树莓派端需同步设置 ROS_DOMAIN_ID=42，否则无法通信
"""

import os
import warnings
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    SetEnvironmentVariable,
    LogInfo  # 新增：打印提示信息
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition  # 新增：条件判断
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径（确保cleanbot_navigation包存在，且rviz目录有对应配置文件）
    try:
        nav_pkg_dir = get_package_share_directory('cleanbot_navigation')
    except Exception as e:
        raise RuntimeError(f"无法找到cleanbot_navigation包：{e}\n请确认包已编译并source")
    
    # RViz配置文件路径
    rviz_config = os.path.join(nav_pkg_dir, 'rviz', 'nav2_default_view.rviz')
    # 校验RViz配置文件是否存在
    if not os.path.exists(rviz_config):
        warnings.warn(f"RViz配置文件不存在：{rviz_config}，将使用默认配置")
        rviz_config = ''  # 空值则RViz使用默认配置
    
    # 启动参数定义
    use_sim_time = LaunchConfiguration('use_sim_time')
    host_ip = LaunchConfiguration('host_ip')
    raspberry_pi_ip = LaunchConfiguration('raspberry_pi_ip')
    
    # 声明参数（修正默认值+补充说明）
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_host_ip = DeclareLaunchArgument(
        'host_ip',
        default_value='192.168.1.200',  # 修正：示例主机IP，需替换为实际值
        description='主机PC的局域网IP地址（必填，如192.168.1.200）'
    )
    
    declare_raspberry_pi_ip = DeclareLaunchArgument(
        'raspberry_pi_ip',
        default_value='192.168.1.100',  # 修正：示例树莓派IP，需替换为实际值
        description='树莓派的局域网IP地址（仅用于提示，ROS2无需设置MASTER_URI）'
    )
    
    # 核心环境变量配置（ROS2分布式通信关键）
    # 1. 设置DDS通信域ID（必须和树莓派端一致）
    set_domain_id = SetEnvironmentVariable(
        'ROS_DOMAIN_ID',
        '42'
    )
    
    # 2. 设置主机IP（让树莓派能识别主机的RViz节点，ROS2核心配置）
    set_host_ip = SetEnvironmentVariable(
        'ROS_IP',
        host_ip  # 使用传入的主机IP参数
    )
    
    # 3. 禁用localhost仅通信（确保跨机器通信）
    set_localhost_only = SetEnvironmentVariable(
        'ROS_LOCALHOST_ONLY',
        'false'
    )
    
    # 新增：打印关键配置提示（方便排查）
    print_config = LogInfo(
        msg=PythonExpression([
            '"=== 远程RViz配置 ===\\n"',
            '"主机IP："', host_ip, '\\n"',
            '"树莓派IP："', raspberry_pi_ip, '\\n"',
            '"通信域ID：42\\n"',
            '"请确保树莓派端设置 ROS_DOMAIN_ID=42 并启动导航程序 ===\\n"'
        ])
    )
    
    # 启动RViz2节点（修正：配置文件不存在时不传入-d参数）
    rviz_args = ['-d', rviz_config] if rviz_config else []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_remote',
        arguments=rviz_args,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        ros_arguments=['--log-level', 'WARN']
    )
    
    # 构建启动描述
    ld = LaunchDescription()
    
    # 第一步：设置环境变量（必须在所有节点前执行）
    ld.add_action(set_domain_id)
    ld.add_action(set_host_ip)
    ld.add_action(set_localhost_only)
    
    # 第二步：声明参数
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_host_ip)
    ld.add_action(declare_raspberry_pi_ip)
    
    # 第三步：打印配置提示
    ld.add_action(print_config)
    
    # 第四步：启动RViz2
    ld.add_action(rviz_node)
    
    return ld

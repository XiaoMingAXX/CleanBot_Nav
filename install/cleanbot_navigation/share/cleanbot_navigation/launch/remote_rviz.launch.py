#!/usr/bin/env python3


import os
import warnings
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    LogInfo  # 添加日志输出功能
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition  # 添加条件判断功能
from launch_ros.actions import Node


def generate_launch_description():
    # 获取cleanbot_navigation功能包路径，用于定位rviz配置文件
    try:
        nav_pkg_dir = get_package_share_directory('cleanbot_navigation')
    except Exception as e:
        raise RuntimeError(f"无法找到cleanbot_navigation功能包: {e}\n请先执行 source 命令编译工作空间")

    # RViz配置文件路径
    rviz_config = os.path.join(nav_pkg_dir, 'rviz', 'nav2_default_view.rviz')
    # 检查配置文件是否存在
    if not os.path.exists(rviz_config):
        warnings.warn(f"RViz配置文件路径不存在: {rviz_config}，将使用默认配置启动RViz")
        rviz_config = ''  # 空配置使用RViz默认设置

    # 定义启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    host_ip = LaunchConfiguration('host_ip')
    raspberry_pi_ip = LaunchConfiguration('raspberry_pi_ip')

    # 声明启动参数（带默认值和描述）
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_host_ip = DeclareLaunchArgument(
        'host_ip',
        default_value='192.168.1.200',  # PC端IP地址，需根据实际网络修改
        description='PC端的IP地址，用于ROS2节点通信（默认：192.168.1.200）'
    )

    declare_raspberry_pi_ip = DeclareLaunchArgument(
        'raspberry_pi_ip',
        default_value='192.168.1.100',  # 树莓派IP地址，需根据实际网络修改
        description='树莓派的IP地址，ROS2 MASTER_URI指向该地址'
    )

    # 设置ROS2网络通信环境变量
    # 1. 设置DDS域ID，确保和树莓派端保持一致
    set_domain_id = SetEnvironmentVariable(
        'ROS_DOMAIN_ID',
        '42'
    )

    # 2. 设置PC端IP，用于和树莓派ROS2节点通信
    set_host_ip = SetEnvironmentVariable(
        'ROS_IP',
        host_ip  # 使用声明的host_ip参数
    )

    # 3. 关闭仅本地通信限制
    set_localhost_only = SetEnvironmentVariable(
        'ROS_LOCALHOST_ONLY',
        'false'
    )

    # 输出配置信息（方便调试）
    print_config = LogInfo(
        msg=PythonExpression([
            '"=== 远程RViz配置信息 ===\\n"',
            '"PC端IP: "', host_ip, '\\n"',
            '"树莓派IP: "', raspberry_pi_ip, '\\n"',
            '"通信域ID: 42\\n"',
            '"请确保树莓派端已设置 ROS_DOMAIN_ID=42 并启动相关节点 ===\\n"'
        ])
    )

    # 启动RViz2（如果配置文件存在则加载，否则使用默认配置）
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

    # 添加环境变量设置
    ld.add_action(set_domain_id)
    #ld.add_action(set_host_ip)  # 如需手动指定PC IP可取消注释
    ld.add_action(set_localhost_only)

    # 添加参数声明
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_host_ip)
    ld.add_action(declare_raspberry_pi_ip)

    # 添加配置信息输出
    ld.add_action(print_config)

    # 添加RViz2节点
    ld.add_action(rviz_node)

    return ld
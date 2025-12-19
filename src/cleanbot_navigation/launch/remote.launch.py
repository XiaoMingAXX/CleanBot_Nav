#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 基础环境变量设置
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')
    set_localhost = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', 'false')
    nav_pkg_dir = get_package_share_directory('cleanbot_navigation')
    # 声明参数
    declare_host_ip = DeclareLaunchArgument(
        'host_ip', default_value='192.168.1.200'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')


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
    
    # 构建启动描述
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(set_domain_id)
    ld.add_action(set_localhost)
    #ld.add_action(declare_host_ip)
    ld.add_action(rviz_node)
    
    return ld

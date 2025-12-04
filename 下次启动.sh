#!/bin/bash
# CleanBot 启动脚本

# 切换到工作空间目录
cd /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws

# 停止 ROS2 daemon（避免缓存问题）
ros2 daemon stop 2>/dev/null

# 清理 DDS 缓存（避免网络冲突）
rm -rf /dev/shm/fastrtps* 2>/dev/null
rm -rf ~/.fastdds/ 2>/dev/null

# 设置独立的 ROS_DOMAIN_ID（避免与其他 ROS2 进程冲突）
export ROS_DOMAIN_ID=42

# 加载 ROS2 环境
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 启动 CleanBot 系统
echo "🤖 启动 CleanBot 系统..."
echo "📡 ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""
ros2 launch cleanbot_control cleanbot.launch.py

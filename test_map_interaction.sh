#!/bin/bash
# 快速测试前端导航地图交互功能

echo "======================================"
echo "  前端导航地图交互功能快速测试"
echo "======================================"
echo ""

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 错误: ROS2环境未加载"
    echo "请先运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

# 检查工作空间
WORKSPACE_DIR="$HOME/桌面/MOON/Electronic/CleanBot_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "❌ 错误: 工作空间不存在: $WORKSPACE_DIR"
    exit 1
fi

cd "$WORKSPACE_DIR"

# 加载工作空间环境
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ 已加载工作空间环境"
else
    echo "❌ 错误: 未找到 install/setup.bash"
    echo "请先编译: colcon build --symlink-install"
    exit 1
fi

echo ""
echo "======================================"
echo "  启动测试环境"
echo "======================================"
echo ""
echo "即将启动以下节点:"
echo "  1. USB通讯节点 (模拟模式)"
echo "  2. 手动控制节点"
echo "  3. Web控制节点 (端口: 8080)"
echo "  4. 导航系统 (模拟模式)"
echo ""
echo "浏览器访问: http://localhost:8080"
echo ""
echo "======================================"
echo "  测试功能清单"
echo "======================================"
echo ""
echo "✅ 1. 障碍物信息显示 (白色方格)"
echo "✅ 2. TF位姿和自动居中 (机器人居中)"
echo "✅ 3. 缩放控制 (+/-/📍 按钮)"
echo "✅ 4. 地图保存和加载"
echo "✅ 5. AMCL初始化 (两次点击)"
echo "✅ 6. 导航目标点发布 (绿色圆圈)"
echo "✅ 7. 清扫路径显示 (黄色虚线)"
echo "✅ 8. 目标点区分 (绿色=用户,红色=执行)"
echo "✅ 9. 清扫进度显示 (左上角)"
echo "✅ 10. 模式切换限制 (清扫时禁止点击)"
echo ""
echo "详细测试步骤请查看: 测试前端地图交互.md"
echo ""
echo "按 Ctrl+C 停止测试"
echo ""
sleep 2

# 启动launch文件
echo "🚀 正在启动系统..."
ros2 launch cleanbot_control cleanbot.launch.py sim_mode:=true &
LAUNCH_PID=$!

# 等待启动
sleep 3

echo ""
echo "✅ 系统已启动 (PID: $LAUNCH_PID)"
echo ""
echo "======================================"
echo "  访问Web界面"
echo "======================================"
echo ""
echo "浏览器打开: http://localhost:8080"
echo ""
echo "或运行: xdg-open http://localhost:8080"
echo ""
echo "======================================"
echo "  常用测试命令"
echo "======================================"
echo ""
echo "# 查看地图数据"
echo "ros2 topic echo /map --once"
echo ""
echo "# 查看规划路径"
echo "ros2 topic echo /cleaning/planned_path --once"
echo ""
echo "# 查看TF变换"
echo "ros2 run tf2_ros tf2_echo map base_link"
echo ""
echo "# 查看AMCL初始位姿"
echo "ros2 topic echo /initialpose --once"
echo ""
echo "# 查看导航目标点"
echo "ros2 topic echo /goal_pose --once"
echo ""
echo "======================================"
echo ""

# 等待用户中断
wait $LAUNCH_PID


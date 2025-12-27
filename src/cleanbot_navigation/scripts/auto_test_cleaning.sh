#!/bin/bash
# 清扫功能自动测试脚本
# 依次测试沿边、弓形、全屋模式

echo "=================================="
echo "CleanBot 清扫功能自动测试"
echo "=================================="
echo ""

# 检查ROS2环境
if ! command -v ros2 &> /dev/null; then
    echo "错误: 未找到ROS2环境"
    exit 1
fi

# 检查是否source了工作空间
if ! ros2 pkg prefix cleanbot_navigation &> /dev/null; then
    echo "错误: 未找到cleanbot_navigation包"
    echo "请先运行: source install/setup.bash"
    exit 1
fi

echo "提示: 请确保已启动以下系统："
echo "  终端1: ros2 launch cleanbot_navigation navigation_sim.launch.py"
echo "  终端2: ros2 launch cleanbot_navigation cleaning_sim.launch.py"
echo ""
read -p "按Enter继续，或Ctrl+C取消..."
echo ""

# 测试1: 沿边清扫
echo "=== 测试1: 沿边清扫 ==="
echo "设置清扫区域（2x2米方形）..."
ros2 topic pub --once /cleaning/area_cmd std_msgs/msg/String \
  'data: "{\"points\": [[0.5, 0.5], [2.5, 0.5], [2.5, 2.5], [0.5, 2.5]]}"'

sleep 2

echo "启动沿边清扫..."
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 1'

echo "请在RViz中观察沿边清扫过程（30秒）..."
sleep 30

echo ""
echo "=== 测试2: 弓形清扫 ==="
echo "设置清扫区域（2x2米方形）..."
ros2 topic pub --once /cleaning/area_cmd std_msgs/msg/String \
  'data: "{\"points\": [[0.5, 0.5], [2.5, 0.5], [2.5, 2.5], [0.5, 2.5]]}"'

sleep 2

# 清除覆盖地图
echo "清除覆盖地图..."
ros2 service call /cleaning/clear_coverage std_srvs/srv/Trigger

sleep 1

echo "启动弓形清扫..."
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 2'

echo "请在RViz中观察弓形清扫过程（30秒）..."
sleep 30

echo ""
echo "=== 测试3: 全屋清扫 ==="
echo "设置清扫区域（更大区域）..."
ros2 topic pub --once /cleaning/area_cmd std_msgs/msg/String \
  'data: "{\"points\": [[-0.5, -0.5], [3.0, -0.5], [3.0, 3.0], [-0.5, 3.0]]}"'

sleep 2

# 清除覆盖地图
echo "清除覆盖地图..."
ros2 service call /cleaning/clear_coverage std_srvs/srv/Trigger

sleep 1

echo "启动全屋清扫（弓形+沿边）..."
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 3'

echo "请在RViz中观察全屋清扫过程（60秒）..."
sleep 60

echo ""
echo "=================================="
echo "测试完成！"
echo "=================================="
echo ""
echo "查看测试结果："
echo "  1. 在RViz中检查覆盖地图（Coverage Map）"
echo "  2. 查看清扫路径是否正确执行"
echo "  3. 检查机器人是否按预期轨迹移动"
echo ""







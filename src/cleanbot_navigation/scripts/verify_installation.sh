#!/bin/bash
# 清扫系统安装验证脚本

echo "=================================="
echo "CleanBot 清扫系统安装验证"
echo "=================================="
echo ""

EXIT_CODE=0

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} 文件存在: $1"
        return 0
    else
        echo -e "${RED}✗${NC} 文件缺失: $1"
        EXIT_CODE=1
        return 1
    fi
}

check_executable() {
    if [ -x "$1" ]; then
        echo -e "${GREEN}✓${NC} 可执行: $1"
        return 0
    else
        echo -e "${YELLOW}⚠${NC} 不可执行: $1 (不影响功能)"
        return 0
    fi
}

check_ros_package() {
    if ros2 pkg prefix "$1" &> /dev/null; then
        echo -e "${GREEN}✓${NC} ROS包已安装: $1"
        return 0
    else
        echo -e "${RED}✗${NC} ROS包未找到: $1"
        EXIT_CODE=1
        return 1
    fi
}

check_node() {
    local pkg=$1
    local node=$2
    if ros2 pkg executables "$pkg" | grep -q "^$node$"; then
        echo -e "${GREEN}✓${NC} 节点已注册: $pkg/$node"
        return 0
    else
        echo -e "${RED}✗${NC} 节点未注册: $pkg/$node"
        EXIT_CODE=1
        return 1
    fi
}

echo "1. 检查Python节点文件"
echo "-----------------------------------"
check_file "src/cleanbot_navigation/cleanbot_navigation/cleaning_area_manager.py"
check_file "src/cleanbot_navigation/cleanbot_navigation/coverage_path_planner.py"
check_file "src/cleanbot_navigation/cleanbot_navigation/coverage_map_manager.py"
check_file "src/cleanbot_navigation/cleanbot_navigation/cleaning_task_manager.py"
check_file "src/cleanbot_navigation/cleanbot_navigation/area_point_collector.py"

echo ""
echo "2. 检查配置文件"
echo "-----------------------------------"
check_file "src/cleanbot_navigation/behavior_trees/cleaning_navigate_to_pose.xml"
check_file "src/cleanbot_navigation/behavior_trees/cleaning_navigate_through_poses.xml"
check_file "src/cleanbot_navigation/rviz/cleaning_view.rviz"
check_file "src/cleanbot_navigation/launch/cleaning_sim.launch.py"

echo ""
echo "3. 检查测试脚本"
echo "-----------------------------------"
check_file "src/cleanbot_navigation/scripts/test_cleaning.py"
check_executable "src/cleanbot_navigation/scripts/test_cleaning.py"
check_file "src/cleanbot_navigation/scripts/auto_test_cleaning.sh"
check_executable "src/cleanbot_navigation/scripts/auto_test_cleaning.sh"

echo ""
echo "4. 检查文档"
echo "-----------------------------------"
check_file "src/cleanbot_navigation/README_CLEANING.md"
check_file "src/cleanbot_navigation/FRONTEND_INTEGRATION.md"
check_file "QUICK_START_CLEANING.md"
check_file "CLEANING_SYSTEM_SUMMARY.md"

echo ""
echo "5. 检查ROS包和节点"
echo "-----------------------------------"
check_ros_package "cleanbot_navigation"

if ros2 pkg prefix cleanbot_navigation &> /dev/null; then
    check_node "cleanbot_navigation" "cleaning_area_manager"
    check_node "cleanbot_navigation" "coverage_path_planner"
    check_node "cleanbot_navigation" "coverage_map_manager"
    check_node "cleanbot_navigation" "cleaning_task_manager"
    check_node "cleanbot_navigation" "area_point_collector"
fi

echo ""
echo "=================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}✓ 验证通过！所有文件和节点都已正确安装${NC}"
    echo ""
    echo "下一步："
    echo "  1. 启动导航系统: ros2 launch cleanbot_navigation navigation_sim.launch.py"
    echo "  2. 启动清扫系统: ros2 launch cleanbot_navigation cleaning_sim.launch.py"
    echo "  3. 运行测试: python3 src/cleanbot_navigation/scripts/test_cleaning.py"
    echo ""
    echo "详细使用说明请查看: QUICK_START_CLEANING.md"
else
    echo -e "${RED}✗ 验证失败！请检查上述错误${NC}"
    echo ""
    echo "建议："
    echo "  1. 确保已编译: colcon build --packages-select cleanbot_navigation"
    echo "  2. 确保已source: source install/setup.bash"
    echo "  3. 检查文件权限和路径"
fi
echo "=================================="

exit $EXIT_CODE












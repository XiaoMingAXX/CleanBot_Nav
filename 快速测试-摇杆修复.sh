#!/bin/bash

echo "============================================"
echo "摇杆控制修复 - 快速测试脚本"
echo "============================================"
echo ""

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${YELLOW}[提示] 请按照以下步骤测试摇杆修复${NC}"
echo ""

# 1. 检查系统是否运行
echo -e "${BLUE}步骤1: 检查系统状态${NC}"
echo "----------------------------------------------"
cd /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws
source install/setup.bash

if ros2 node list 2>/dev/null | grep -q "web_control_node"; then
    echo -e "${GREEN}✓ web_control_node 正在运行${NC}"
else
    echo -e "${RED}✗ web_control_node 未运行${NC}"
    echo ""
    echo "请先启动系统："
    echo "  ros2 launch cleanbot_control cleanbot.launch.py"
    exit 1
fi
echo ""

# 2. 打开终端监测
echo -e "${BLUE}步骤2: 在新终端监测cmd_vel话题${NC}"
echo "----------------------------------------------"
echo "请打开一个新终端，运行以下命令："
echo ""
echo -e "${YELLOW}  ros2 topic echo /diff_drive_controller/cmd_vel${NC}"
echo ""
echo "或者监测发布频率："
echo ""
echo -e "${YELLOW}  ros2 topic hz /diff_drive_controller/cmd_vel${NC}"
echo ""
read -p "按回车键继续..."
echo ""

# 3. 打开浏览器
echo -e "${BLUE}步骤3: 打开浏览器并强制刷新${NC}"
echo "----------------------------------------------"
echo "1. 打开浏览器：http://localhost:8080"
echo "2. ${RED}重要${NC}: 按 ${YELLOW}Ctrl + Shift + R${NC} 强制刷新（清除缓存）"
echo "3. 等待WebSocket连接成功"
echo ""
read -p "按回车键继续..."
echo ""

# 4. 测试摇杆持续控制
echo -e "${BLUE}步骤4: 测试摇杆持续控制${NC}"
echo "----------------------------------------------"
echo "测试项目："
echo ""
echo "  1) 拖动摇杆到某个位置并${YELLOW}保持不动${NC}"
echo "  2) 观察监测终端的cmd_vel话题"
echo "  3) 应该看到：${GREEN}持续收到相同的速度命令（约每100ms一次）${NC}"
echo "  4) 观察机器人：${GREEN}应该平滑持续移动，不会断断续续${NC}"
echo "  5) 松开摇杆：${GREEN}机器人应该立即停止${NC}"
echo ""
echo "预期结果："
echo "  ${GREEN}✓${NC} 话题持续发布（10 Hz）"
echo "  ${GREEN}✓${NC} 机器人平滑移动"
echo "  ${GREEN}✓${NC} 松开后立即停止"
echo ""
echo "如果出现问题："
echo "  ${RED}✗${NC} 话题断断续续 → 可能浏览器缓存未清除"
echo "  ${RED}✗${NC} 机器人时动时停 → 检查WebSocket连接"
echo ""
read -p "按回车键继续..."
echo ""

# 5. 测试不同速度
echo -e "${BLUE}步骤5: 测试不同速度${NC}"
echo "----------------------------------------------"
echo "测试场景："
echo ""
echo "  场景A: ${YELLOW}静止（摇杆在中心）${NC}"
echo "    - 监测终端应该：${GREEN}无消息发布${NC}"
echo "    - 机器人应该：${GREEN}静止不动${NC}"
echo ""
echo "  场景B: ${YELLOW}匀速前进（摇杆向上）${NC}"
echo "    - 监测终端应该：${GREEN}持续发布 linear.x > 0${NC}"
echo "    - 机器人应该：${GREEN}平滑前进${NC}"
echo ""
echo "  场景C: ${YELLOW}匀速旋转（摇杆向左）${NC}"
echo "    - 监测终端应该：${GREEN}持续发布 angular.z > 0${NC}"
echo "    - 机器人应该：${GREEN}平滑旋转${NC}"
echo ""
echo "  场景D: ${YELLOW}前进+旋转（摇杆斜向）${NC}"
echo "    - 监测终端应该：${GREEN}持续发布 linear.x > 0 且 angular.z > 0${NC}"
echo "    - 机器人应该：${GREEN}平滑弧线运动${NC}"
echo ""
read -p "按回车键继续..."
echo ""

# 6. 测试模式切换失败反馈
echo -e "${BLUE}步骤6: 测试模式切换失败反馈${NC}"
echo "----------------------------------------------"
echo "测试场景："
echo ""
echo "  1) 确保地图服务器${YELLOW}未启动${NC}"
echo "  2) 在浏览器中点击${YELLOW}\"导航\"${NC}模式"
echo "  3) 观察状态显示"
echo ""
echo "预期结果："
echo "  ${GREEN}✓${NC} 状态显示：⏳ 切换中... → ${RED}❌ 切换失败${NC}"
echo "  ${GREEN}✓${NC} 日志显示：${RED}❌ 模式切换失败: 地图服务器未启动${NC}"
echo "  ${GREEN}✓${NC} 3秒后恢复：${GREEN}✅ 就绪${NC}"
echo ""
read -p "按回车键继续..."
echo ""

# 7. 性能检查
echo -e "${BLUE}步骤7: 性能检查${NC}"
echo "----------------------------------------------"
echo "在监测终端检查发布频率："
echo ""
echo -e "${YELLOW}  ros2 topic hz /diff_drive_controller/cmd_vel${NC}"
echo ""
echo "预期结果："
echo "  - 摇杆移动时：${GREEN}约 10 Hz${NC}"
echo "  - 摇杆静止时：${GREEN}0 Hz（无发布）${NC}"
echo ""
read -p "按回车键继续..."
echo ""

# 8. 浏览器控制台检查
echo -e "${BLUE}步骤8: 浏览器控制台检查${NC}"
echo "----------------------------------------------"
echo "在浏览器中按 F12 打开开发者工具，在Console中输入："
echo ""
echo -e "${YELLOW}  typeof startMapServer${NC}"
echo "  应该返回: ${GREEN}\"function\"${NC}"
echo ""
echo -e "${YELLOW}  typeof refreshMapList${NC}"
echo "  应该返回: ${GREEN}\"function\"${NC}"
echo ""
echo -e "${YELLOW}  joystickEnabled${NC}"
echo "  手动模式应该返回: ${GREEN}true${NC}"
echo "  导航模式应该返回: ${GREEN}false${NC}"
echo ""
read -p "按回车键继续..."
echo ""

# 总结
echo "============================================"
echo -e "${GREEN}测试完成！${NC}"
echo "============================================"
echo ""
echo "如果所有测试都通过："
echo "  ${GREEN}✓${NC} 摇杆控制应该平滑流畅"
echo "  ${GREEN}✓${NC} 模式切换失败有明确提示"
echo "  ${GREEN}✓${NC} 地图管理功能正常"
echo ""
echo "如果仍有问题，请检查："
echo "  1. 浏览器缓存是否清除（Ctrl+Shift+R）"
echo "  2. WebSocket连接是否正常"
echo "  3. ROS节点是否正常运行"
echo ""
echo "详细文档请查看："
echo "  - 摇杆问题修复说明.md"
echo "  - 浏览器缓存问题解决方案.md"
echo ""



#!/bin/bash

echo "=========================================="
echo "导航系统问题修复验证脚本"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. 检查地图文件
echo -e "${YELLOW}[1/5] 检查地图文件...${NC}"
if [ -d ~/cleanbot_maps ]; then
    map_count=$(ls ~/cleanbot_maps/*.yaml 2>/dev/null | wc -l)
    if [ $map_count -gt 0 ]; then
        echo -e "${GREEN}✓ 找到 $map_count 个地图文件${NC}"
        ls -lh ~/cleanbot_maps/*.yaml
    else
        echo -e "${RED}✗ 地图目录存在但没有地图文件${NC}"
        echo "  请先进行建图并保存地图"
    fi
else
    echo -e "${RED}✗ 地图目录不存在${NC}"
    echo "  创建目录: mkdir -p ~/cleanbot_maps"
    mkdir -p ~/cleanbot_maps
fi
echo ""

# 2. 检查编译状态
echo -e "${YELLOW}[2/5] 检查编译状态...${NC}"
cd /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws
if [ -f install/setup.bash ]; then
    echo -e "${GREEN}✓ 工作空间已编译${NC}"
else
    echo -e "${RED}✗ 工作空间未编译${NC}"
    echo "  运行: colcon build"
    exit 1
fi
echo ""

# 3. 检查修改的文件
echo -e "${YELLOW}[3/5] 检查修改的文件...${NC}"
files_to_check=(
    "src/cleanbot_control/web/static/control.js"
    "install/cleanbot_control/share/cleanbot_control/web/static/control.js"
)

for file in "${files_to_check[@]}"; do
    if [ -f "$file" ]; then
        # 检查是否包含修复的代码
        if grep -q "lastSentLinear" "$file"; then
            echo -e "${GREEN}✓ $file - 包含摇杆优化代码${NC}"
        else
            echo -e "${RED}✗ $file - 缺少摇杆优化代码${NC}"
            echo "  需要重新编译: colcon build --packages-select cleanbot_control"
        fi
        
        if grep -q "setTimeout.*refreshMapList" "$file"; then
            echo -e "${GREEN}✓ $file - 包含地图列表修复代码${NC}"
        else
            echo -e "${RED}✗ $file - 缺少地图列表修复代码${NC}"
        fi
    else
        echo -e "${RED}✗ 文件不存在: $file${NC}"
    fi
done
echo ""

# 4. 检查ROS节点是否运行
echo -e "${YELLOW}[4/5] 检查ROS节点状态...${NC}"
source install/setup.bash

if ros2 node list 2>/dev/null | grep -q "web_control_node"; then
    echo -e "${GREEN}✓ web_control_node 正在运行${NC}"
    
    # 检查话题
    if ros2 topic list 2>/dev/null | grep -q "cmd_vel"; then
        echo -e "${GREEN}✓ cmd_vel 话题存在${NC}"
    else
        echo -e "${YELLOW}⚠ cmd_vel 话题不存在${NC}"
    fi
    
    if ros2 topic list 2>/dev/null | grep -q "navigation/mode_cmd"; then
        echo -e "${GREEN}✓ navigation/mode_cmd 话题存在${NC}"
    else
        echo -e "${YELLOW}⚠ navigation/mode_cmd 话题不存在${NC}"
    fi
else
    echo -e "${YELLOW}⚠ web_control_node 未运行${NC}"
    echo "  启动系统: ros2 launch cleanbot_control cleanbot.launch.py"
fi
echo ""

# 5. 生成测试报告
echo -e "${YELLOW}[5/5] 生成测试清单...${NC}"
cat << EOF

========================================
手动测试清单
========================================

1. 启动系统（如果未运行）:
   cd /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws
   source install/setup.bash
   ros2 launch cleanbot_control cleanbot.launch.py

2. 打开浏览器（隐私模式）:
   http://localhost:8080

3. 测试地图列表:
   [ ] WebSocket连接成功后，自动刷新地图列表
   [ ] 下拉框显示: cleanbot_map_20251215_201805
   [ ] 点击"刷新列表"按钮，列表正常更新

4. 测试摇杆控制:
   [ ] 手动模式：拖动摇杆，机器人移动
   [ ] 建图模式：摇杆可用，可以驾驶建图
   [ ] 导航模式：摇杆禁用，无法拖动
   [ ] 日志显示正确的摇杆状态

5. 检查浏览器控制台（F12）:
   [ ] 无错误信息
   [ ] WebSocket连接正常
   [ ] 消息收发正常

========================================
调试命令（如果有问题）
========================================

# 在浏览器控制台输入：
ws.readyState           # 应该返回 1 (OPEN)
joystickEnabled         # 根据模式返回 true/false
currentNavigationMode   # 0=手动, 1=建图, 2=导航

# 在终端输入：
ros2 topic echo /diff_drive_controller/cmd_vel
ros2 topic echo /navigation/mode_status
ros2 node info /web_control_node

========================================
EOF

echo -e "${GREEN}验证脚本执行完成！${NC}"
echo "请按照上述测试清单进行手动测试"
echo ""



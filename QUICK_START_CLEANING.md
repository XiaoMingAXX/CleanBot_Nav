# CleanBot 智能清扫系统 - 快速启动指南

## 系统功能

实现了四种智能清扫模式：

1. **待机模式**: 单点导航（现有功能）
2. **沿边模式**: 在指定区域内沿障碍物边缘清扫
3. **弓形模式**: 在指定区域内弓字形全覆盖清扫
4. **全屋模式**: 自动执行全屋弓形覆盖 + 沿边清扫

## 快速启动

### 1. 编译工作空间

```bash
cd /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws
colcon build --packages-select cleanbot_navigation
source install/setup.bash
```

### 2. 启动仿真环境

打开终端1，启动导航系统：

```bash
source install/setup.bash
ros2 launch cleanbot_navigation navigation_sim.launch.py
```

### 3. 启动清扫系统

打开终端2，启动清扫节点：

```bash
source install/setup.bash
ros2 launch cleanbot_navigation cleaning_sim.launch.py
```

这会启动：
- 清扫区域管理节点
- 路径规划节点
- 覆盖地图管理节点
- 清扫任务管理节点
- 区域点击收集节点
- RViz2可视化界面（清扫视图）

## 使用方法

### 方法1: 在RViz中交互式定义清扫区域

1. 在RViz工具栏选择 **"Publish Point"** 工具
2. 在地图上依次点击4个点定义清扫区域（按顺序点击）
3. 点击第4个点后，清扫区域会自动显示为绿色边框

### 方法2: 使用测试脚本

打开终端3：

```bash
source install/setup.bash
python3 src/cleanbot_navigation/scripts/test_cleaning.py
```

按提示选择测试模式：
- 1: 测试沿边清扫
- 2: 测试弓形清扫
- 3: 测试全屋清扫

### 方法3: 使用命令行

#### 定义清扫区域

```bash
# 定义一个2x2米的方形区域
ros2 topic pub --once /cleaning/area_cmd std_msgs/msg/String \
  'data: "{\"points\": [[0.5, 0.5], [2.5, 0.5], [2.5, 2.5], [0.5, 2.5]]}"'
```

#### 切换清扫模式

```bash
# 沿边模式
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 1'

# 弓形模式
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 2'

# 全屋模式
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 3'

# 待机模式（恢复单点导航）
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 0'
```

## RViz可视化说明

清扫视图包含以下元素：

| 显示项 | 颜色 | 说明 |
|--------|------|------|
| Cleaning Area | 绿色边框 + 红点 | 清扫区域边界和顶点 |
| Boustrophedon Path | 黄色 | 弓形覆盖路径 |
| Edge Following Path | 青色 | 沿边清扫路径 |
| Coverage Map | 黑色半透明 | 已清扫区域 |
| Robot Pose | 黄色箭头 | 机器人当前位置 |
| Global Path | 绿色 | 全局规划路径 |
| Local Path | 红色 | 局部规划路径 |

## 完整测试流程示例

### 测试1: 小区域弓形清扫

```bash
# 终端3
source install/setup.bash

# 定义清扫区域
ros2 topic pub --once /cleaning/area_cmd std_msgs/msg/String \
  'data: "{\"points\": [[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]]}"'

# 等待2秒，确保区域已设置
sleep 2

# 启动弓形清扫
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 2'
```

在RViz中观察：
1. 绿色框显示清扫区域
2. 黄色路径显示弓形轨迹
3. 机器人沿路径移动
4. 黑色区域显示已清扫部分

### 测试2: 沿边清扫

```bash
# 定义清扫区域
ros2 topic pub --once /cleaning/area_cmd std_msgs/msg/String \
  'data: "{\"points\": [[0.5, 0.5], [2.5, 0.5], [2.5, 2.5], [0.5, 2.5]]}"'

sleep 2

# 启动沿边清扫
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 1'
```

### 测试3: 全屋清扫（弓形+沿边）

```bash
# 定义更大的清扫区域
ros2 topic pub --once /cleaning/area_cmd std_msgs/msg/String \
  'data: "{\"points\": [[-1.0, -1.0], [4.0, -1.0], [4.0, 4.0], [-1.0, 4.0]]}"'

sleep 2

# 启动全屋清扫
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 3'
```

机器人会：
1. 先执行弓形覆盖整个区域
2. 完成后自动切换到沿边清扫
3. 沿边缘再走一圈

## 调试和监控

### 查看节点状态

```bash
ros2 node list | grep cleaning
```

应该看到：
- /cleaning_area_manager
- /coverage_path_planner
- /coverage_map_manager
- /cleaning_task_manager
- /area_point_collector

### 查看话题

```bash
# 查看清扫相关话题
ros2 topic list | grep cleaning

# 监控清扫模式状态
ros2 topic echo /cleaning/mode_status

# 监控任务信息
ros2 topic echo /cleaning/task_info
```

### 查看路径

```bash
# 查看弓形路径
ros2 topic echo /cleaning/boustrophedon_path

# 查看沿边路径
ros2 topic echo /cleaning/edge_path
```

### 手动触发路径计算

```bash
# 计算弓形路径
ros2 service call /cleaning/compute_boustrophedon_path std_srvs/srv/Trigger

# 计算沿边路径
ros2 service call /cleaning/compute_edge_path std_srvs/srv/Trigger

# 清除覆盖地图
ros2 service call /cleaning/clear_coverage std_srvs/srv/Trigger
```

## 参数调整

编辑 launch 文件中的参数来调整清扫行为：

```python
# src/cleanbot_navigation/launch/cleaning_sim.launch.py

coverage_path_planner = Node(
    ...
    parameters=[{
        'robot_width': 0.35,      # 机器人宽度（米）
        'path_spacing': 0.3,      # 弓形路径间距（米）- 减小可提高覆盖密度
        'wall_distance': 0.2      # 沿边距离（米）
    }]
)

coverage_map_manager = Node(
    ...
    parameters=[{
        'robot_radius': 0.2       # 机器人半径（米）- 影响覆盖地图显示
    }]
)
```

修改后重新编译：

```bash
colcon build --packages-select cleanbot_navigation
source install/setup.bash
```

## 常见问题

### Q1: 机器人不移动

**A**: 检查导航系统是否完全启动：
```bash
ros2 lifecycle get /bt_navigator
# 应该返回 "active [3]"
```

如果不是active状态，切换到导航模式：
```bash
ros2 topic pub --once /navigation/mode_cmd std_msgs/msg/UInt8 'data: 2'
```

### Q2: 看不到清扫区域

**A**: 确保：
1. area_point_collector 节点正在运行
2. 已经点击了4个点
3. RViz中 "Cleaning Area" 显示项已启用

### Q3: 路径规划失败

**A**: 检查：
1. 地图是否已加载：`ros2 topic echo /map --once`
2. 清扫区域是否已设置：`ros2 topic echo /cleaning/area_polygon --once`
3. 路径规划服务是否可用：`ros2 service list | grep cleaning`

### Q4: 覆盖地图不显示

**A**: 
1. 确保机器人在移动
2. 检查AMCL是否发布位置：`ros2 topic hz /amcl_pose`
3. 在RViz中启用 "Coverage Map" 显示项

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                   前端/用户界面                          │
│  (发送工作模式、清扫区域)                                │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│            cleaning_task_manager                        │
│        (清扫任务管理 - 协调四种模式)                     │
└──┬─────────────────┬──────────────────┬────────────────┘
   │                 │                  │
   │                 │                  │
┌──▼──────────┐ ┌───▼────────────┐ ┌──▼──────────────┐
│  待机模式    │ │ coverage_path  │ │ coverage_map    │
│  (单点导航)  │ │   _planner     │ │   _manager      │
│             │ │ (路径规划)      │ │ (覆盖地图)       │
└─────────────┘ └───┬────────────┘ └─────────────────┘
                    │
           ┌────────┴────────┐
           │                 │
    ┌──────▼──────┐   ┌─────▼──────┐
    │  弓形路径    │   │  沿边路径   │
    │  规划        │   │  规划       │
    └─────────────┘   └────────────┘
                    │
           ┌────────▼────────┐
           │   Nav2 导航栈    │
           │ (路径执行)       │
           └─────────────────┘
```

## 下一步

- 调整参数优化清扫效果
- 测试不同形状和大小的清扫区域
- 观察覆盖地图的实时更新
- 尝试在复杂环境中的清扫任务

详细文档请参考：`src/cleanbot_navigation/README_CLEANING.md`











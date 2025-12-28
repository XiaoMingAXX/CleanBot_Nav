# CleanBot 智能清扫导航系统

## 功能概述

实现了四种清扫工作模式：

1. **待机模式 (Standby)**: 单点导航，发点即走
2. **沿边模式 (Edge Following)**: 在指定区域内沿障碍物边缘清扫
3. **弓形模式 (Boustrophedon)**: 在指定区域内弓字形全覆盖清扫
4. **全屋模式 (Auto Whole House)**: 先弓形覆盖全屋，再沿边清扫边缘

## 系统架构

### 核心节点

1. **cleaning_area_manager**: 清扫区域管理
   - 接收用户指定的四个点定义清扫区域
   - 发布清扫区域多边形和可视化标记

2. **coverage_path_planner**: 覆盖路径规划
   - 计算弓形路径（按间距平行扫描）
   - 计算沿边路径（沿障碍物边缘）
   - 提供路径计算服务

3. **coverage_map_manager**: 覆盖地图管理
   - 实时记录机器人已清扫区域
   - 发布覆盖地图用于可视化

4. **cleaning_task_manager**: 清扫任务管理
   - 协调四种工作模式
   - 管理导航任务执行
   - 处理任务状态切换

5. **area_point_collector**: 区域点击收集
   - 在RViz中通过点击收集清扫区域的4个点
   - 自动发布清扫区域命令

## 使用方法

### 1. 启动仿真环境

首先启动机器人仿真和导航系统：

```bash
# 终端1: 启动仿真环境和导航
ros2 launch cleanbot_navigation navigation_sim.launch.py
```

### 2. 启动清扫系统

```bash
# 终端2: 启动清扫节点
ros2 launch cleanbot_navigation cleaning_sim.launch.py
```

### 3. 定义清扫区域

在RViz中：
1. 选择 "Publish Point" 工具（工具栏）
2. 在地图上依次点击4个点定义清扫区域（顺时针或逆时针）
3. 点击第4个点后，清扫区域会自动发布并显示为绿色边框

或者通过命令行：

```bash
# 发布清扫区域（单位：米）
ros2 topic pub --once /cleaning/area_cmd std_msgs/msg/String \
  'data: "{\"points\": [[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]]}"'
```

### 4. 切换清扫模式

```bash
# 待机模式 (0)
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 0'

# 沿边模式 (1) - 需要先定义清扫区域
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 1'

# 弓形模式 (2) - 需要先定义清扫区域
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 2'

# 全屋模式 (3) - 自动弓形+沿边
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 3'
```

### 5. RViz可视化

清扫视图包含以下显示：

- **Cleaning Area**: 绿色边框 + 红色顶点，显示清扫区域
- **Boustrophedon Path**: 黄色路径，弓形覆盖轨迹
- **Edge Following Path**: 青色路径，沿边清扫轨迹
- **Coverage Map**: 半透明覆盖，显示已清扫区域（黑色）
- **Robot Pose**: 黄色箭头，机器人当前位置

## 话题接口

### 订阅话题

- `/map` (nav_msgs/OccupancyGrid): 环境地图
- `/amcl_pose` (geometry_msgs/PoseStamped): 机器人定位
- `/cleaning/area_point` (geometry_msgs/PointStamped): 清扫区域点击点

### 发布话题

- `/cleaning/area_polygon` (geometry_msgs/PolygonStamped): 清扫区域多边形
- `/cleaning/area_markers` (visualization_msgs/MarkerArray): 清扫区域可视化
- `/cleaning/boustrophedon_path` (nav_msgs/Path): 弓形路径
- `/cleaning/edge_path` (nav_msgs/Path): 沿边路径
- `/cleaning/coverage_map` (nav_msgs/OccupancyGrid): 覆盖地图
- `/cleaning/mode_status` (std_msgs/UInt8): 当前清扫模式

### 服务

- `/cleaning/compute_boustrophedon_path` (std_srvs/Trigger): 计算弓形路径
- `/cleaning/compute_edge_path` (std_srvs/Trigger): 计算沿边路径
- `/cleaning/clear_coverage` (std_srvs/Trigger): 清除覆盖地图

## 参数配置

### coverage_path_planner 参数

```yaml
robot_width: 0.35      # 机器人宽度（米）
path_spacing: 0.3      # 弓形路径间距（米）
wall_distance: 0.2     # 沿边距离（米）
```

### coverage_map_manager 参数

```yaml
robot_radius: 0.2      # 机器人半径（米）
```

## 测试示例

### 1. 弓形清扫测试

```bash
# 1. 定义清扫区域（2x2米方形）
ros2 topic pub --once /cleaning/area_cmd std_msgs/msg/String \
  'data: "{\"points\": [[0.5, 0.5], [2.5, 0.5], [2.5, 2.5], [0.5, 2.5]]}"'

# 2. 切换到弓形模式
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 2'
```

### 2. 沿边清扫测试

```bash
# 1. 定义清扫区域
ros2 topic pub --once /cleaning/area_cmd std_msgs/msg/String \
  'data: "{\"points\": [[0.5, 0.5], [2.5, 0.5], [2.5, 2.5], [0.5, 2.5]]}"'

# 2. 切换到沿边模式
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 1'
```

### 3. 全屋清扫测试

```bash
# 切换到全屋模式（会自动执行弓形+沿边）
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 'data: 3'
```

## 行为树配置

系统使用了两个行为树：

- `cleaning_navigate_to_pose.xml`: 单点导航（待机模式）
- `cleaning_navigate_through_poses.xml`: 多点导航（弓形/沿边/全屋模式）

这些行为树基于Nav2默认行为树，支持重规划和恢复行为。

## 故障排查

### 问题1: 路径规划失败

**原因**: 未收到地图或清扫区域
**解决**: 确保导航模式已启动，且已定义清扫区域

### 问题2: 机器人不移动

**原因**: Nav2未完全启动或Action服务不可用
**解决**: 检查Nav2节点状态，确保所有lifecycle节点已激活

### 问题3: 覆盖地图不显示

**原因**: 机器人位置未更新或地图未初始化
**解决**: 确保AMCL已启动并发布位置

## 扩展开发

### 添加新的清扫模式

1. 在 `cleaning_task_manager.py` 中添加新模式常量
2. 在 `coverage_path_planner.py` 中实现路径规划算法
3. 在任务管理器中添加模式执行逻辑

### 优化路径规划算法

当前实现是简化版本，可以优化：

- 弓形路径：添加障碍物避让
- 沿边路径：使用更精确的边缘检测算法
- 全局优化：考虑起始位置，选择最优路径顺序










# RViz可视化配置说明

## 概述

本文档说明如何在RViz中配置和查看清扫系统的完整可视化，包括：
- 🗺️ **静态地图**
- 🔴 **外边界**（红色线框）
- 🟡 **障碍物边界**（黄色线框）
- 🟢 **路径航点**（绿色小球）
- 🔵 **路径连线**（青色线条）
- 🔴 **起点**（红色大球）
- 🔵 **终点**（蓝色大球）

---

## RViz配置步骤

### 1. 添加地图显示

```
Add → By display type → Map
Topic: /map
Color Scheme: map
```

### 2. 添加边界可视化

```
Add → By topic → /cleaning/boundary_markers → MarkerArray
```

**显示效果：**
- 外边界：红色粗线框，线宽0.08m
- 障碍物边界：黄色线框，线宽0.06m

### 3. 添加路径航点可视化

```
Add → By topic → /cleaning/waypoint_markers → MarkerArray
```

**显示效果：**
- 路径航点：绿色小球（直径0.1m）
- 路径连线：青蓝色线条（线宽0.05m）
- 起点标记：红色大球（直径0.3m）
- 终点标记：蓝色大球（直径0.3m）

### 4. 添加完整路径显示

```
Add → By topic → /cleaning/planned_path → Path
Color: 橙色 (可选)
```

### 5. 添加机器人位置

```
Add → By topic → /amcl_pose → PoseWithCovarianceStamped
Shape: Arrow
Color: 红色
```

### 6. 添加导航目标点

```
Add → By topic → /goal_pose → PoseStamped
Shape: Arrow
Color: 绿色
```

---

## 完整RViz配置文件示例

将以下内容保存为 `cleanbot_cleaning_visualization.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Time
    Name: Time
  - Class: rviz_common/Tool Properties
    Name: Tool Properties

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Value: true

    - Class: rviz_default_plugins/MarkerArray
      Name: 边界标记
      Namespaces:
        outer_boundary: true
        obstacles: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /cleaning/boundary_markers
      Value: true

    - Class: rviz_default_plugins/MarkerArray
      Name: 路点标记
      Namespaces:
        waypoints: true
        path_line: true
        start_point: true
        end_point: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /cleaning/waypoint_markers
      Value: true

    - Alpha: 1.0
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 255; 170; 0
      Name: 清扫路径
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /cleaning/planned_path
      Value: true

    - Alpha: 1.0
      Axes Length: 0.5
      Axes Radius: 0.05
      Class: rviz_default_plugins/PoseWithCovariance
      Color: 255; 0; 0
      Name: 机器人位置
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /amcl_pose
      Value: true

  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
    - Class: rviz_default_plugins/PublishPoint
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: TopDownOrtho
      Near Clip Distance: 0.01
      Scale: 100
      Target Frame: map
      X: 0
      Y: 0
    Saved: ~
```

---

## 地图预处理算法流程

本系统采用基于OpenCV的地图预处理算法，将栅格地图转换为多边形表示：

### 步骤1: 读取栅格地图并二值化
- 读取OccupancyGrid地图
- 二值化：自由空间=255（白），障碍物=0（黑）

### 步骤2: 按机器尺寸膨胀
- 根据机器人半径（0.15m）进行形态学膨胀
- 确保机器人不会与障碍物碰撞

### 步骤3: 开运算和闭运算去除噪声
- **开运算**：先腐蚀后膨胀，去除小的噪声点
- **闭运算**：先膨胀后腐蚀，填充小的空洞

### 步骤4: findContours找到所有多边形轮廓
- 使用OpenCV的`findContours`函数
- 提取所有封闭轮廓

### 步骤5: 识别外边界和障碍物
- **外边界**：面积最大的轮廓
- **障碍物**：其他所有轮廓（面积>10像素²）

---

## 三种清扫模式的路径规划

### 1. 沿边清扫（Edge Cleaning）

**原理：** 将外边界向内偏移35cm作为清扫路径

**流程：**
1. 地图预处理，提取外边界
2. 对外边界进行内偏移（腐蚀操作）
3. 沿着内偏移边界绕一圈
4. 检查并绕开障碍物

**可视化：**
- 红色线框显示原始外边界
- 绿色路点显示内偏移35cm的清扫路径

---

### 2. 弓形清扫（Boustrophedon）

**原理：** 在外边界内规划弓字形覆盖路线

**流程：**
1. 地图预处理，提取外边界
2. 计算边界框
3. 根据条带宽度生成弓字形扫描线
4. 逐行扫描，交替方向
5. 检查点是否在多边形内且安全

**参数：**
- 条带宽度 = 机器人直径（0.3m）
- 采样间距 = 分辨率 × 5.0

**可视化：**
- 绿色路点显示弓字形路径
- 路径自动避开障碍物

---

### 3. 自动全屋清扫（Auto Coverage）⭐

**原理：** Cell Decomposition + 蚁群算法求解TSP

**完整算法流程：**

#### 步骤1: 地图预处理
- 提取外边界和所有障碍物
- 转换为多边形表示

#### 步骤2: Cell Decomposition（多边形分块）
- 使用**垂直扫描线方法**分块
- 原理：用一条自下而上的直线段，根据线段和多边形的交点数量变化来触发分块
- 每个Cell约2.5m宽
- 每个Cell有：
  - 顶点集合（矩形区域）
  - 入口点（左侧中点）
  - 出口点（右侧中点）
  - 覆盖路径（弓形）

#### 步骤3: 为每个Cell生成覆盖路径
- 自动选择最优方向（沿短边扫描）
- 生成弓字形路径
- 检查每个点的安全性

#### 步骤4: 蚁群算法求解TSP
- **节点图构建：** 每个Cell的进口和出口作为节点
- **Cost计算：** 欧氏距离
  - 起点到各Cell入口
  - Cell出口到其他Cell入口
- **ACO参数：**
  - 蚂蚁数量：20
  - 迭代次数：100
  - α（信息素权重）：1.0
  - β（启发式因子权重）：2.0
  - ρ（挥发率）：0.5
  - Q（信息素强度）：100.0
- **算法流程：**
  1. 初始化信息素矩阵
  2. 每只蚂蚁基于信息素和启发式因子构建路径
  3. 更新最优解
  4. 信息素挥发
  5. 信息素更新（按路径长度）
  6. 重复迭代

#### 步骤5: 恢复完整路径
- 从起点到第一个Cell入口
- 按最优顺序遍历所有Cell
- Cell之间的转移路径（出口→入口）
- 连接所有子路径

**输出统计：**
- 总航点数
- 覆盖子区域数
- 总路径长度（米）
- 最优访问顺序

**可视化：**
- 红色线框：外边界
- 黄色线框：障碍物
- 绿色小球：所有路径航点
- 青蓝色线条：路径连线
- 红色大球：起点
- 蓝色大球：终点

---

## 终端输出示例

### 沿边清扫
```
[cleaning_task] ========== 沿边清扫路径规划 ==========
[cleaning_task] [1/5] 从静态地图提取并内偏移边界...
[cleaning_task]   [地图预处理] 步骤1: 读取地图并二值化...
[cleaning_task]   [地图预处理] 步骤2: 按机器尺寸膨胀...
[cleaning_task]   [地图预处理] 步骤3: 开运算和闭运算去除噪声...
[cleaning_task]   [地图预处理] 步骤4: findContours找到所有多边形轮廓...
[cleaning_task]   找到3个轮廓
[cleaning_task]   ✓ 外边界: 24个顶点, 面积=125432.50像素²
[cleaning_task]   ✓ 障碍物: 2个
[cleaning_task] ✓ 内偏移边界提取完成: 22个顶点, 偏移距离=0.35m
[cleaning_task] [2/5] 寻找最近边界点...
[cleaning_task] ✓ 最近边界点索引=5, 距离=1.23m
[cleaning_task] [3/5] 生成起点到边界的路径...
[cleaning_task] [4/5] 生成沿边清扫路径（绕开障碍物）...
[cleaning_task] ⚠ 跳过了15个不安全的路径点（有障碍物）
[cleaning_task] [5/5] 沿边路径规划完成
[cleaning_task] ✅ 路径生成完成: 187个航点 (耗时0.45秒)
[cleaning_task] 📍 发布完整清扫路径: 187个航点
[cleaning_task] 🎨 发布路点可视化标记
[cleaning_task] 🎨 发布边界可视化: 外边界+2个障碍物
```

### 自动全屋清扫
```
[cleaning_task] ========== 自动全屋覆盖路径规划 ==========
[cleaning_task] [1/5] 地图预处理: 提取多边形边界和障碍物...
[cleaning_task]   [地图预处理] 步骤1: 读取地图并二值化...
[cleaning_task]   [地图预处理] 步骤2: 按机器尺寸膨胀...
[cleaning_task]   [地图预处理] 步骤3: 开运算和闭运算去除噪声...
[cleaning_task]   [地图预处理] 步骤4: findContours找到所有多边形轮廓...
[cleaning_task]   找到5个轮廓
[cleaning_task]   ✓ 外边界: 28个顶点, 面积=150234.75像素²
[cleaning_task]   ✓ 障碍物: 4个
[cleaning_task] ✓ 提取到外边界: 28个顶点, 障碍物: 4个
[cleaning_task] [2/5] 执行Cell Decomposition分块算法...
[cleaning_task]   边界框: x[-5.23, 12.45], y[-3.12, 8.76]
[cleaning_task]   预计分块数量: 7 (每块宽度约2.50m)
[cleaning_task]   Cell Decomposition完成: 生成7个有效分块
[cleaning_task] ✓ 分解完成: 共7个子区域
[cleaning_task] [3/5] 为每个子区域生成弓形覆盖路径...
[cleaning_task] ✓ 所有子区域路径生成完成
[cleaning_task] [4/5] 使用蚁群算法求解TSP，优化子区域访问顺序...
[cleaning_task]   蚁群算法求解TSP: 7个城市, 20只蚂蚁, 100次迭代
[cleaning_task]   迭代[10/100]: 当前最优路径长度=45.32m
[cleaning_task]   迭代[20/100]: 当前最优路径长度=42.18m
[cleaning_task]   迭代[30/100]: 当前最优路径长度=40.25m
[cleaning_task]   迭代[40/100]: 当前最优路径长度=38.94m
[cleaning_task]   迭代[50/100]: 当前最优路径长度=38.12m
[cleaning_task]   迭代[60/100]: 当前最优路径长度=37.65m
[cleaning_task]   迭代[70/100]: 当前最优路径长度=37.45m
[cleaning_task]   迭代[80/100]: 当前最优路径长度=37.23m
[cleaning_task]   迭代[90/100]: 当前最优路径长度=37.05m
[cleaning_task]   迭代[100/100]: 当前最优路径长度=36.98m
[cleaning_task]   蚁群算法求解完成: 最优路径长度=36.98m
[cleaning_task] ✓ TSP求解完成, 最优访问顺序: [0 2 4 6 5 3 1]
[cleaning_task] [5/5] 连接所有子路径生成完整覆盖路径...
[cleaning_task] ========== 自动全屋覆盖路径规划完成 ==========
[cleaning_task] ✓ 总航点数: 1245
[cleaning_task] ✓ 覆盖子区域数: 7
[cleaning_task] ✓ 总路径长度: 156.78m
[cleaning_task] 📍 发布完整清扫路径: 1245个航点
[cleaning_task] 🎨 发布路点可视化标记
[cleaning_task] 🎨 发布边界可视化: 外边界+4个障碍物
```

### 清扫进度输出
```
[cleaning_task] ============================================================
[cleaning_task]   ✓ 完成目标点 [125/1245]
[cleaning_task]   进度: [████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░] 10.0%
[cleaning_task]   剩余: 1120个航点
[cleaning_task] ============================================================
[cleaning_task] 📊 清扫进度: 10.0%
```

---

## 启动命令

### 1. 启动仿真环境
```bash
ros2 launch cleanbot_navigation navigation_sim.launch.py
```

### 2. 启动RViz（使用配置文件）
```bash
rviz2 -d ~/Desktop/MOON/Electronic/CleanBot_ws/cleanbot_cleaning_visualization.rviz
```

### 3. 发送清扫命令

#### 沿边清扫
```bash
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 "{data: 1}"
```

#### 弓形清扫
```bash
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 "{data: 2}"
```

#### 自动全屋清扫
```bash
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 "{data: 3}"
```

#### 停止清扫
```bash
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 "{data: 0}"
```

---

## 话题列表

### 发布的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/cleaning/task_info` | `std_msgs/String` | 任务状态信息 |
| `/cleaning/progress` | `std_msgs/Float32` | 清扫进度（0-100%） |
| `/cleaning/planned_path` | `nav_msgs/Path` | 完整清扫路径 |
| `/cleaning/waypoint_markers` | `visualization_msgs/MarkerArray` | 路点可视化标记 |
| `/cleaning/boundary_markers` | `visualization_msgs/MarkerArray` | 边界可视化标记 |

### 订阅的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/cleaning/mode_cmd` | `std_msgs/UInt8` | 清扫模式命令 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 机器人当前位置 |

---

## 参数配置

参数文件位置：`src/cleanbot_navigation/config/cleaning_task_params.yaml`

```yaml
cleaning_task:
  ros__parameters:
    waypoint_spacing: 0.5        # 路点间距（米）
    robot_radius: 0.15           # 机器人半径（米）
    edge_offset: 0.35            # 沿边偏移距离（米）
    coverage_stripe_width: 0.3   # 弓形条带宽度（米）
    corner_radius: 0.3           # 转角圆弧半径（米）
    queue_size: 2                # 目标点队列大小（滑动窗口）
```

---

## 故障排查

### 1. 地图服务不可用
**现象：** `地图服务不可用`

**解决方法：**
```bash
# 检查map_server是否运行
ros2 node list | grep map_server

# 检查地图话题
ros2 topic list | grep /map

# 重新启动导航
ros2 launch cleanbot_navigation navigation_sim.launch.py
```

### 2. 路径规划失败
**现象：** `路径生成失败`

**可能原因：**
- 地图未正确加载
- 机器人位置未初始化
- 地图中没有有效的自由空间

**解决方法：**
- 检查/map话题是否发布
- 在RViz中设置初始位置（2D Pose Estimate）
- 检查地图质量

### 3. 路点可视化不显示
**现象：** RViz中看不到绿色路点

**解决方法：**
```bash
# 检查话题是否发布
ros2 topic echo /cleaning/waypoint_markers

# 在RViz中刷新MarkerArray显示
# 或重新添加 /cleaning/waypoint_markers
```

---

## 算法性能

### 计算复杂度

- **地图预处理：** O(W×H) - W和H为地图宽高
- **Cell Decomposition：** O(W×H) - 扫描线算法
- **单Cell路径生成：** O(N) - N为Cell内路点数
- **ACO求解TSP：** O(I×A×N²) 
  - I：迭代次数（100）
  - A：蚂蚁数量（20）
  - N：Cell数量
- **总体复杂度：** O(W×H + I×A×N²)

### 典型性能指标

以10m×10m房间为例：

| 清扫模式 | 路径生成时间 | 航点数量 | 路径长度 |
|---------|------------|---------|---------|
| 沿边清扫 | ~0.3秒 | 150-250 | 30-40m |
| 弓形清扫 | ~0.4秒 | 300-500 | 60-80m |
| 自动全屋 | ~1.5秒 | 800-1500 | 120-180m |

---

## 参考资料

1. **Cell Decomposition算法：**
   - Choset, H. (2001). "Coverage path planning: The boustrophedon cellular decomposition"

2. **蚁群算法：**
   - Dorigo, M., & Stützle, T. (2004). "Ant Colony Optimization"

3. **TSP问题：**
   - Traveling Salesman Problem - 组合优化经典问题

4. **OpenCV形态学操作：**
   - OpenCV Documentation: Morphological Transformations

---

**作者：** CleanBot开发团队  
**版本：** v2.0  
**更新日期：** 2025-12-26







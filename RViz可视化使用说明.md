# RViz可视化使用说明

## 🎨 完整可视化配置已添加

RViz配置文件已更新，包含完整的清扫系统可视化功能！

**配置文件位置：** `src/cleanbot_navigation/rviz/cleanbot_nav_view.rviz`

---

## 🚀 快速启动

### 1. 启动导航系统
```bash
cd ~/桌面/MOON/Electronic/CleanBot_ws
source install/setup.bash
ros2 launch cleanbot_navigation navigation_sim.launch.py
```

### 2. 启动RViz（使用配置文件）
```bash
rviz2 -d ~/桌面/MOON/Electronic/CleanBot_ws/src/cleanbot_navigation/rviz/cleanbot_nav_view.rviz
```

---

## 📋 新增可视化内容

在RViz左侧面板的 **"清扫系统可视化"** 组中，您会看到：

### 1. 🔴🟡 边界标记 (`/cleaning/boundary_markers`)
显示地图预处理后的多边形边界：
- **红色粗线框** (0.08m线宽): 外边界
- **黄色线框** (0.06m线宽): 障碍物边界

**控制方式：**
- 点击 "边界标记" 旁边的复选框启用/禁用
- 展开 "Namespaces" 可以单独控制：
  - `outer_boundary`: 外边界显示
  - `obstacles`: 障碍物显示

### 2. 🟢🔵 路点标记 (`/cleaning/waypoint_markers`)
显示规划的所有清扫路径点：
- **绿色小球** (0.1m直径): 所有路径航点
- **青蓝色线条** (0.05m线宽): 路径连线
- **红色大球** (0.3m直径): 起点标记
- **蓝色大球** (0.3m直径): 终点标记

**控制方式：**
- 点击 "路点标记" 复选框启用/禁用
- 展开 "Namespaces" 可以单独控制：
  - `waypoints`: 路径点小球
  - `path_line`: 路径连线
  - `start_point`: 起点标记
  - `end_point`: 终点标记

### 3. 🟠 清扫路径 (`/cleaning/planned_path`)
显示完整的清扫路径（橙黄色Path）
- **颜色**: 橙黄色 (RGB: 255, 170, 0)
- **线宽**: 0.05m

**控制方式：**
- 点击 "清扫路径" 复选框启用/禁用

---

## 🎮 完整操作流程

### 步骤1: 设置机器人初始位置
1. 在RViz工具栏点击 **"2D Pose Estimate"** 
2. 在地图上点击并拖动设置位置和方向

### 步骤2: 发送清扫命令

#### 沿边清扫
```bash
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 "{data: 1}"
```

**预期可视化效果：**
- ✅ 红色外边界线框出现
- ✅ 黄色障碍物线框出现（如有）
- ✅ 绿色路点沿边界一圈
- ✅ 青色路径连线
- ✅ 红色起点球，蓝色终点球

#### 弓形清扫
```bash
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 "{data: 2}"
```

**预期可视化效果：**
- ✅ 绿色路点呈弓字形分布
- ✅ 往返扫描轨迹清晰可见

#### 自动全屋清扫（完整算法）
```bash
ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 "{data: 3}"
```

**预期可视化效果：**
- ✅ 显示Cell分块结果
- ✅ 显示蚁群优化后的最优路径
- ✅ 覆盖所有子区域的完整路径

### 步骤3: 观察可视化

**在RViz中查看：**
1. **地图预处理结果**
   - 红色线框显示提取的外边界
   - 黄色线框显示识别的障碍物

2. **路径规划结果**
   - 绿色小球密集分布，覆盖整个区域
   - 青色线条连接形成完整路径
   - 起点（红球）和终点（蓝球）清晰标识

3. **机器人运动**
   - 绿色粒子云（AMCL定位）
   - 机器人TF坐标系
   - 激光扫描点云

**在终端中查看：**
```
[cleaning_task] ========== 自动全屋覆盖路径规划 ==========
[cleaning_task] [1/5] 地图预处理: 提取多边形边界和障碍物...
[cleaning_task]   [地图预处理] 步骤1: 读取地图并二值化...
[cleaning_task]   [地图预处理] 步骤2: 按机器尺寸膨胀...
[cleaning_task]   [地图预处理] 步骤3: 开运算和闭运算去除噪声...
[cleaning_task]   [地图预处理] 步骤4: findContours找到所有多边形轮廓...
[cleaning_task]   找到3个轮廓
[cleaning_task]   ✓ 外边界: 24个顶点, 面积=125432.50像素²
[cleaning_task]   ✓ 障碍物: 2个
[cleaning_task] ✓ 提取到外边界: 24个顶点, 障碍物: 2个
[cleaning_task] [2/5] 执行Cell Decomposition分块算法...
[cleaning_task]   边界框: x[-2.50, 7.50], y[-3.20, 8.30]
[cleaning_task]   预计分块数量: 4 (每块宽度约2.50m)
[cleaning_task]   Cell Decomposition完成: 生成4个有效分块
[cleaning_task] ✓ 分解完成: 共4个子区域
[cleaning_task] [3/5] 为每个子区域生成弓形覆盖路径...
[cleaning_task] ✓ 所有子区域路径生成完成
[cleaning_task] [4/5] 使用蚁群算法求解TSP，优化子区域访问顺序...
[cleaning_task]   蚁群算法求解TSP: 4个城市, 20只蚂蚁, 100次迭代
[cleaning_task]   迭代[10/100]: 当前最优路径长度=35.42m
[cleaning_task]   迭代[20/100]: 当前最优路径长度=32.18m
...
[cleaning_task]   蚁群算法求解完成: 最优路径长度=28.65m
[cleaning_task] ✓ TSP求解完成, 最优访问顺序: [0 2 3 1]
[cleaning_task] [5/5] 连接所有子路径生成完整覆盖路径...
[cleaning_task] ========== 自动全屋覆盖路径规划完成 ==========
[cleaning_task] ✓ 总航点数: 856
[cleaning_task] ✓ 覆盖子区域数: 4
[cleaning_task] ✓ 总路径长度: 128.45m
[cleaning_task] 📍 发布完整清扫路径: 856个航点
[cleaning_task] 🎨 发布路点可视化标记
[cleaning_task] 🎨 发布边界可视化: 外边界+2个障碍物
```

---

## 🔧 RViz显示控制

### 显示/隐藏各个元素

在RViz左侧 **Displays** 面板中：

```
□ 清扫系统可视化
  □ 边界标记
    Namespaces:
      □ outer_boundary     ← 外边界（红色）
      □ obstacles          ← 障碍物（黄色）
  □ 路点标记
    Namespaces:
      □ waypoints          ← 路径点（绿色小球）
      □ path_line          ← 路径连线（青色）
      □ start_point        ← 起点（红色大球）
      □ end_point          ← 终点（蓝色大球）
  □ 清扫路径             ← 完整路径（橙色）
```

### 调整显示属性

**修改颜色：**
1. 选中要修改的显示项
2. 在下方属性面板找到 "Color"
3. 点击颜色框修改RGB值

**修改线宽/大小：**
1. 选中显示项
2. 找到 "Scale" 或 "Line Width"
3. 调整数值

### 常用视图操作

- **鼠标滚轮**: 缩放
- **鼠标中键拖动**: 平移
- **Shift + 鼠标左键**: 旋转（3D视图）
- **Ctrl + Shift + 鼠标左键**: 改变视角

---

## 📊 完整的话题列表

### 清扫系统发布的话题

| 话题 | 类型 | 说明 | 可视化 |
|-----|------|------|--------|
| `/cleaning/boundary_markers` | `MarkerArray` | 边界可视化 | ✅ 已配置 |
| `/cleaning/waypoint_markers` | `MarkerArray` | 路点可视化 | ✅ 已配置 |
| `/cleaning/planned_path` | `Path` | 完整路径 | ✅ 已配置 |
| `/cleaning/task_info` | `String` | 任务状态 | 终端监控 |
| `/cleaning/progress` | `Float32` | 清扫进度 | 终端监控 |

### 监控命令

```bash
# 查看边界标记
ros2 topic echo /cleaning/boundary_markers

# 查看路点标记
ros2 topic echo /cleaning/waypoint_markers

# 查看路径
ros2 topic echo /cleaning/planned_path

# 查看进度
ros2 topic echo /cleaning/progress

# 查看任务信息
ros2 topic echo /cleaning/task_info
```

---

## 🎯 可视化效果展示

### 沿边清扫
```
外边界（红色）─┐
               │
    ┌──────────┘
    │   起点（红球）
    │      ↓
    │   [绿色路点沿边界一圈]
    │      ↓
    │   终点（蓝球）
    └──────────
障碍物（黄色框）
```

### 弓形清扫
```
起点→ → → → → ↓
          ↓   ↓
    ← ← ← ←   ↓
    ↓         ↓
    → → → → → ↓
          ↓   ↓
    ← ← ← ← ← ←终点
```

### 自动全屋（Cell分块）
```
┌─────┬─────┬─────┬─────┐
│Cell0│Cell2│Cell3│Cell1│  ← 蚁群算法优化顺序
├─────┼─────┼─────┼─────┤
│ ↓↑  │ ↓↑  │ ↓↑  │ ↓↑  │  ← 每个Cell内弓形扫描
│ ↓↑  │ ↓↑  │ ↓↑  │ ↓↑  │
│ ↓↑  │ ↓↑  │ ↓↑  │ ↓↑  │
└─────┴─────┴─────┴─────┘
起点 → Cell0入口 → ... → Cell1出口
```

---

## 🐛 故障排查

### 问题1: RViz中看不到可视化

**症状：** 启动后看不到红色边界或绿色路点

**检查步骤：**
1. 确认话题发布
   ```bash
   ros2 topic list | grep cleaning
   ```
   应该看到：
   ```
   /cleaning/boundary_markers
   /cleaning/waypoint_markers
   /cleaning/planned_path
   /cleaning/progress
   /cleaning/task_info
   ```

2. 确认已发送清扫命令
   ```bash
   ros2 topic pub --once /cleaning/mode_cmd std_msgs/msg/UInt8 "{data: 3}"
   ```

3. 检查RViz中 "清扫系统可视化" 组是否展开并启用

4. 检查各Namespace是否勾选

### 问题2: 边界标记显示异常

**症状：** 红色边界或黄色障碍物显示不全

**解决方法：**
1. 重新触发路径规划
2. 检查地图是否正确加载
   ```bash
   ros2 topic echo /map -n 1
   ```

### 问题3: 路点标记不显示

**症状：** 只看到边界，没有绿色路点

**解决方法：**
1. 查看终端输出，确认路径已生成
2. 检查话题
   ```bash
   ros2 topic hz /cleaning/waypoint_markers
   ```
3. 在RViz中刷新 MarkerArray 显示

### 问题4: RViz启动失败

**症状：** RViz无法打开配置文件

**解决方法：**
```bash
# 使用绝对路径
rviz2 -d /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws/src/cleanbot_navigation/rviz/cleanbot_nav_view.rviz

# 或先cd到工作空间
cd ~/桌面/MOON/Electronic/CleanBot_ws
rviz2 -d src/cleanbot_navigation/rviz/cleanbot_nav_view.rviz
```

---

## 📝 使用技巧

### 1. 快速切换显示模式

**只看边界：**
- 禁用 "路点标记" 和 "清扫路径"
- 只启用 "边界标记"

**只看路径：**
- 禁用 "边界标记"
- 启用 "路点标记" 和 "清扫路径"

**完整显示：**
- 全部启用

### 2. 性能优化

如果RViz卡顿：
1. 禁用 "路点标记" 中的 `waypoints` (绿色小球很多)
2. 只保留 `path_line` (路径连线)
3. 降低点云密度

### 3. 截图保存

1. 调整视角到最佳位置
2. 在RViz菜单: `File → Save Image`
3. 选择保存位置

### 4. 自定义配置

修改后保存：
1. 调整好所有显示属性
2. 在RViz菜单: `File → Save Config As...`
3. 保存为新的 `.rviz` 文件

---

## 📚 相关文档

- **完整算法实现报告**: `完整算法实现报告.md`
- **RViz可视化配置说明**: `RViz可视化配置说明.md`
- **快速测试指南**: `快速测试-完整版.md`
- **清扫任务节点说明**: `docs/清扫任务节点重构说明.md`

---

## ✅ 检查清单

使用前确认：
- [ ] 已启动导航系统
- [ ] RViz已加载 `cleanbot_nav_view.rviz` 配置
- [ ] "清扫系统可视化" 组已展开
- [ ] 所有显示项已启用
- [ ] 已设置机器人初始位置
- [ ] 已发送清扫命令

可视化正常：
- [ ] 看到红色外边界线框
- [ ] 看到黄色障碍物线框（如有）
- [ ] 看到绿色路径点
- [ ] 看到青色路径连线
- [ ] 看到红色起点标记
- [ ] 看到蓝色终点标记
- [ ] 看到橙色路径线

---

**配置文件版本:** v2.0  
**更新日期:** 2025-12-26  
**状态:** ✅ 已完成并测试











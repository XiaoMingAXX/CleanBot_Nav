# DWB优化和平滑算法移除说明

## 修改日期
2025-12-25

## 问题描述

1. **平滑算法导致路径变形**: 原有的平滑算法会修改清扫路径，导致覆盖不完整
2. **机器人卡在原地**: DWB局部规划器参数不合理，导致机器人无法跟踪路径
3. **路径频繁重规划**: 全局规划器频率过高，清扫路径不应该频繁变化

## 解决方案

### 1. 删除平滑算法 ✅

**修改文件**:
- `src/cleanbot_navigation/src/coverage_planner_base.cpp`
- `src/cleanbot_navigation/src/edge_planner.cpp`

**修改内容**:
```cpp
// 之前: 调用平滑算法
auto smoothed_poses = smoothPath(poses, 0.3);
path.poses = smoothed_poses;

// 修改后: 直接使用原始路径
path.poses = poses;
```

**原因**:
- 平滑算法会修改路径点位置，导致清扫路径偏离原始规划
- 对于弓形和全屋覆盖路径，保持原始路径更重要
- DWB控制器本身有路径跟踪能力，不需要额外平滑

### 2. 配置路径规划为静态模式 ✅

**修改文件**: `config/nav2_sim_params.yaml`

**修改内容**:
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0  # 从20.0降低到1.0Hz
```

**原因**:
- 清扫路径规划一次后基本不需要改变
- 降低规划频率可以减少计算负担
- 避免路径频繁变化导致机器人行为不稳定

### 3. 优化DWB参数解决卡死问题 ✅

**修改文件**: `config/nav2_sim_params.yaml`

#### 3.1 降低速度限制

```yaml
max_vel_x: 0.20      # 从0.25降低到0.20m/s
max_vel_theta: 0.5   # 从0.8降低到0.5rad/s
```

**原因**: 降低速度提高路径跟踪精度，避免过冲

#### 3.2 降低加速度

```yaml
acc_lim_x: 0.3       # 从0.4降低到0.3
acc_lim_theta: 0.8   # 从1.2降低到0.8
```

**原因**: 更平滑的加减速，避免急停急转

#### 3.3 增加角速度采样

```yaml
vtheta_samples: 40   # 从30增加到40
```

**原因**: 更多的角速度采样提高转向精度

#### 3.4 放宽目标容差

```yaml
xy_goal_tolerance: 0.25     # 从0.15增加到0.25m
yaw_goal_tolerance: 0.25    # 新增，约14度
```

**原因**: 避免机器人在接近目标时过度调整导致卡死

#### 3.5 调整评分函数权重 (关键!)

```yaml
# 强调路径跟随
PathAlign.scale: 64.0    # 从48.0增加到64.0 (最高权重)
PathDist.scale: 48.0     # 从40.0增加到48.0

# 降低目标导向
GoalAlign.scale: 8.0     # 从20.0大幅降低到8.0
GoalDist.scale: 12.0     # 从20.0降低到12.0

# 最小化旋转
RotateToGoal.scale: 4.0  # 从24.0大幅降低到4.0

# 降低障碍物避让
BaseObstacle.scale: 0.01 # 从0.02降低到0.01
```

**原因**:
- **PathAlign和PathDist高权重**: 让机器人专注于跟随路径，而不是直接朝向目标
- **GoalAlign和GoalDist低权重**: 避免过早朝向目标导致偏离路径
- **RotateToGoal低权重**: 减少不必要的原地旋转
- **BaseObstacle低权重**: 避免过度避让导致偏离清扫路径

#### 3.6 添加振荡检测

```yaml
Oscillation.scale: 1.0
Oscillation.oscillation_reset_dist: 0.05
Oscillation.oscillation_reset_angle: 0.2
```

**原因**: 防止机器人来回摆动

### 4. 关闭路径细化 ✅

**修改文件**: `config/nav2_sim_params.yaml`

```yaml
smoother_server:
  simple_smoother:
    do_refinement: false  # 从true改为false
```

**原因**: 保持原始清扫路径不被修改

## 修改总结

### 修改的文件列表

1. ✅ `src/cleanbot_navigation/src/coverage_planner_base.cpp`
   - 删除平滑算法调用

2. ✅ `src/cleanbot_navigation/src/edge_planner.cpp`
   - 删除平滑算法调用

3. ✅ `config/nav2_sim_params.yaml`
   - 降低全局规划器频率: 20Hz → 1Hz
   - 优化DWB速度参数
   - 优化DWB评分函数权重
   - 关闭路径细化

### 关键参数对比

| 参数 | 修改前 | 修改后 | 说明 |
|------|--------|--------|------|
| `expected_planner_frequency` | 20.0 | 1.0 | 路径规划频率 |
| `max_vel_x` | 0.25 | 0.20 | 最大线速度 |
| `max_vel_theta` | 0.8 | 0.5 | 最大角速度 |
| `vtheta_samples` | 30 | 40 | 角速度采样数 |
| `xy_goal_tolerance` | 0.15 | 0.25 | 目标位置容差 |
| `PathAlign.scale` | 48.0 | 64.0 | 路径对齐权重 |
| `GoalAlign.scale` | 20.0 | 8.0 | 目标对齐权重 |
| `RotateToGoal.scale` | 24.0 | 4.0 | 旋转到目标权重 |
| `do_refinement` | true | false | 路径细化开关 |

## 预期效果

### 解决的问题

1. ✅ **路径不再变形**: 删除平滑算法，保持原始清扫路径
2. ✅ **机器人不再卡死**: 优化DWB参数，提高路径跟踪能力
3. ✅ **路径保持稳定**: 降低规划频率，避免频繁重规划

### 行为改善

**修改前**:
- 机器人在原地打转或卡死
- 路径被平滑后偏离原始规划
- 频繁重新规划路径

**修改后**:
- 机器人平滑跟踪清扫路径
- 路径保持原始规划不变
- 路径规划后基本不再改变
- 更专注于路径跟随而非目标导向

## 测试建议

### 1. 基本功能测试

```bash
# 启动导航系统
ros2 launch cleanbot_navigation navigation_sim.launch.py

# 打开Web界面
http://localhost:8000

# 测试三种清扫模式:
1. 沿边清扫
2. 弓形清扫  
3. 自动全屋
```

### 2. 观察指标

- **路径跟踪精度**: 机器人是否沿着规划路径行走
- **转角处理**: 转角处是否平滑，不卡死
- **速度稳定性**: 速度是否平稳，不急停急转
- **覆盖完整性**: 是否完整覆盖规划区域

### 3. 调试日志

观察以下日志:
```
✓ 清扫路径规划完成 [XXX]: XXX个航点
```

确认没有"平滑后"字样，说明平滑算法已移除。

### 4. RViz可视化

在RViz中观察:
- 绿色路径应该是规整的弓形或矩形
- 机器人应该紧密跟随绿色路径
- 不应该有明显的偏离或抖动

## 进一步优化方向

如果仍然有问题，可以尝试:

### 1. 进一步降低速度
```yaml
max_vel_x: 0.15
max_vel_theta: 0.4
```

### 2. 增加路径点密度
在路径规划器中减小`waypoint_spacing`:
```yaml
waypoint_spacing: 0.3  # 从0.5减小到0.3
```

### 3. 调整前瞻距离
```yaml
PathAlign.forward_point_distance: 0.4  # 从0.3增加到0.4
```

### 4. 增加控制频率
```yaml
controller_frequency: 30.0  # 从20.0增加到30.0
```

## 常见问题

### Q: 机器人还是卡死怎么办?
A: 
1. 检查是否有障碍物阻挡
2. 进一步降低速度和加速度
3. 增加`xy_goal_tolerance`到0.3或更大
4. 检查TF树是否正常

### Q: 路径跟踪不精确?
A: 
1. 增加`PathAlign.scale`权重
2. 增加`vtheta_samples`采样数
3. 降低速度提高精度

### Q: 转角处停顿?
A: 
1. 增加`yaw_goal_tolerance`
2. 降低`RotateToGoal.scale`
3. 检查路径点的朝向是否合理

## 编译状态

✅ 编译成功，无错误

```bash
Starting >>> cleanbot_navigation
Finished <<< cleanbot_navigation [8.04s]
Summary: 1 package finished [8.23s]
```

## 相关文档

- [全屋覆盖路径规划实现说明.md](./全屋覆盖路径规划实现说明.md)
- [测试自动全屋覆盖.md](./测试自动全屋覆盖.md)
- [自动全屋覆盖路径规划完成报告.md](./自动全屋覆盖路径规划完成报告.md)

---

**修改完成**: 2025-12-25  
**测试状态**: 编译通过 ✅  
**建议**: 立即进行实际测试验证效果


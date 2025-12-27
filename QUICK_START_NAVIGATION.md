# CleanBot 导航系统快速启动指南

## 📦 安装依赖

### 一键安装脚本

```bash
# 安装Nav2和SLAM工具包
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-rmw-cyclonedds-cpp

# 验证安装
ros2 pkg list | grep -E "navigation2|slam_toolbox"
```

## 🔧 编译导航包

```bash
cd ~/桌面/MOON/Electronic/CleanBot_ws

# 编译导航包
colcon build --packages-select cleanbot_navigation

# 如果修改了控制包，也需重新编译
colcon build --packages-select cleanbot_control

# 刷新环境
source install/setup.bash
```

## 🚀 快速启动

### 启动顺序

**终端1：机器人基础系统**
```bash
source ~/桌面/MOON/Electronic/CleanBot_ws/install/setup.bash
ros2 launch cleanbot_control robot_bringup.launch.py
```

**终端2：Web控制界面**
```bash
source ~/桌面/MOON/Electronic/CleanBot_ws/install/setup.bash
ros2 launch cleanbot_control web_control.launch.py
```

**终端3：导航系统**
```bash
source ~/桌面/MOON/Electronic/CleanBot_ws/install/setup.bash
ros2 launch cleanbot_navigation navigation_bringup.launch.py
```

**终端4（可选）：可视化**
```bash
rviz2
```

## 🗺️ 建图流程

### 步骤1：打开Web界面

浏览器访问：`http://localhost:8080`（如果在机器人上，用机器人IP）

### 步骤2：连接机器人

1. 点击"扫描串口"按钮
2. 选择正确的串口（通常是`/dev/ttyACM0`或`/dev/ttyUSB0`）
3. 点击"连接"
4. 确认USB和WebSocket状态显示为"已连接"

### 步骤3：切换到建图模式

1. 在"导航模式"面板中，点击"建图"按钮
2. 等待系统日志显示"✅ 模式切换成功: mapping"
3. "保存地图"按钮应变为可用状态

### 步骤4：手动驾驶建图

1. 使用摇杆控制机器人在环境中移动
2. **建图技巧**：
   - 慢速移动（提高精度）
   - 尽量形成闭环路径（触发闭环优化）
   - 覆盖所有需要导航的区域
   - 避免快速旋转

### 步骤5：保存地图

1. 完成建图后，点击"保存地图"按钮
2. 查看系统日志确认保存成功
3. 地图保存在：`~/cleanbot_maps/cleanbot_map_<时间戳>.yaml`

## 🧭 导航流程

### 步骤1：准备地图

确认已有保存的地图文件：
```bash
ls ~/cleanbot_maps/
```

### 步骤2：修改启动参数（首次使用）

编辑启动文件或使用命令行参数：
```bash
ros2 launch cleanbot_navigation navigation_bringup.launch.py \
    map:=~/cleanbot_maps/cleanbot_map_20251214_120000.yaml
```

### 步骤3：切换到导航模式

1. 在Web界面点击"导航"按钮
2. 系统会自动：
   - 加载地图
   - 启动AMCL定位
   - 发布初始位姿
   - **禁用摇杆控制**
3. 等待定位收敛（约5秒）

### 步骤4：发送导航目标

**方法1：RViz2（推荐）**
1. 在RViz2中点击"2D Goal Pose"工具
2. 在地图上点击目标位置
3. 拖动鼠标设置目标朝向
4. 松开鼠标发送目标

**方法2：命令行**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, \
    pose: {position: {x: 2.0, y: 1.0, z: 0.0}, \
           orientation: {w: 1.0}}}"
```

### 步骤5：监控导航

- RViz2中观察：
  - 机器人位置（蓝色）
  - 规划路径（黄色）
  - AMCL粒子（绿色箭头）
  - 代价地图（彩色）
- Web界面查看系统日志

## 🎨 RViz2配置

### 基础配置

1. **Fixed Frame**: 设为`map`
2. **添加显示**：
   - `RobotModel`：显示机器人模型
   - `LaserScan`：话题`/scan`
   - `Map`：话题`/map`
   - `Path`：话题`/plan`（全局路径）
   - `Path`：话题`/local_plan`（局部路径）
   - `PoseArray`：话题`/particlecloud`（AMCL粒子）
   - `TF`：显示坐标变换

### 保存配置

```bash
# 保存RViz配置
# File -> Save Config As -> cleanbot_navigation.rviz
```

下次启动RViz2：
```bash
rviz2 -d cleanbot_navigation.rviz
```

## 🔍 故障排查

### 问题1：SLAM节点无法启动

**症状**：切换到建图模式无反应

**解决**：
```bash
# 检查SLAM节点
ros2 node list | grep slam

# 检查生命周期状态
ros2 lifecycle get /slam_toolbox

# 手动激活
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

### 问题2：地图保存失败

**症状**：点击保存地图无反应

**解决**：
```bash
# 创建地图目录
mkdir -p ~/cleanbot_maps
chmod 755 ~/cleanbot_maps

# 手动保存地图测试
ros2 run nav2_map_server map_saver_cli -f ~/cleanbot_maps/test_map
```

### 问题3：AMCL定位不准

**症状**：机器人位置在地图上跳动

**解决方案**：
1. 确保建图起点和导航起点大致相同
2. 在RViz2中使用"2D Pose Estimate"手动设置初始位姿
3. 检查里程计数据：`ros2 topic echo /odometry/filtered`
4. 增加AMCL粒子数（修改`amcl.yaml`中的`max_particles`）

### 问题4：雷达无数据

**症状**：`/scan`话题无消息

**解决**：
```bash
# 检查雷达设备
ls /dev/ttyUSB*

# 如果没有权限
sudo chmod 666 /dev/ttyUSB0

# 重启雷达节点
ros2 lifecycle set /rplidar_node activate
```

### 问题5：导航模式下摇杆仍可用

**症状**：导航时摇杆未被禁用

**解决**：
- 刷新Web页面（Ctrl+F5）
- 检查浏览器控制台是否有JavaScript错误
- 确认WebSocket连接正常

## 📊 性能检查

### 检查话题频率

```bash
# SLAM地图更新
ros2 topic hz /map

# AMCL定位更新
ros2 topic hz /amcl_pose

# 激光扫描
ros2 topic hz /scan

# 里程计
ros2 topic hz /odometry/filtered
```

### 检查系统资源

```bash
# CPU和内存
top

# 节点资源使用
ros2 run ros2_introspection introspection_node
```

## 🎯 高级技巧

### 技巧1：优化建图质量

- 建图时速度控制在0.1-0.2 m/s
- 转弯速度控制在0.3 rad/s以内
- 每个区域至少扫描2次
- 关键区域（门、走廊）多次经过

### 技巧2：提高定位精度

在`config/amcl.yaml`中调整：
```yaml
min_particles: 1000  # 增加粒子数
max_particles: 3000
update_min_d: 0.1    # 减小更新阈值
update_min_a: 0.15
```

### 技巧3：调整导航速度

在`config/nav2_params.yaml`中修改：
```yaml
max_vel_x: 0.5  # 提高最大速度
max_vel_theta: 1.5
acc_lim_x: 0.8  # 提高加速度
```

### 技巧4：多地图管理

```bash
# 为不同区域创建不同地图
~/cleanbot_maps/
├── living_room.yaml
├── kitchen.yaml
└── bedroom.yaml

# 启动时选择地图
ros2 launch cleanbot_navigation navigation_bringup.launch.py \
    map:=~/cleanbot_maps/living_room.yaml
```

## 📝 日常使用检查清单

### 启动前检查
- [ ] 雷达USB线已连接
- [ ] STM32 USB已连接
- [ ] 电池电量充足
- [ ] 环境无重大变化（如果使用已有地图）

### 建图前检查
- [ ] 已切换到建图模式
- [ ] "保存地图"按钮可用
- [ ] RViz2中能看到/scan数据
- [ ] 摇杆控制正常

### 导航前检查
- [ ] 地图文件存在且完整（.yaml + .pgm）
- [ ] 已切换到导航模式
- [ ] RViz2中能看到地图
- [ ] AMCL粒子云已收敛
- [ ] 摇杆已被禁用

## 🆘 紧急情况

### 紧急停止方法

1. **Web界面**：点击"🛑 紧急停止"按钮
2. **物理按钮**：按下机器人上的急停按钮
3. **命令行**：
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/TwistStamped \
     "{twist: {linear: {x: 0}, angular: {z: 0}}}"
   ```

### 切换回手动模式

如果导航出现问题，立即切换回手动模式：
1. Web界面点击"手动"按钮
2. 摇杆恢复可用
3. 手动驾驶机器人到安全位置

## 📞 获取帮助

- 查看日志：Web界面的"系统日志"面板
- 详细文档：`src/cleanbot_navigation/README.md`
- 设计文档：`docs/导航系统设计文档.md`
- 联系支持：3210676508@qq.com

---

**祝使用愉快！** 🤖✨













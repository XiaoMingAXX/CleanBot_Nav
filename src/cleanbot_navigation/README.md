# CleanBot Navigation Package

双轮差速扫地机器人导航系统软件包，集成SLAM建图和Nav2自主导航功能。

## 功能特性

### 1. 三种控制模式
- **手动模式（默认）**：使用Web界面摇杆手动控制机器人移动
- **建图模式**：启动SLAM Toolbox进行实时建图，支持闭环检测
- **导航模式**：基于已保存地图使用AMCL定位和Nav2导航栈进行自主导航

### 2. SLAM建图（SLAM Toolbox）
- 实时2D激光SLAM建图
- 闭环检测和图优化
- 支持大场景建图
- 地图保存和加载功能

### 3. 自主导航（Nav2 + AMCL）
- 基于AMCL的粒子滤波定位
- DWB局部路径规划器
- 全局路径规划（Navfn Planner）
- 动态避障
- 路径平滑

### 4. 模式管理
- 生命周期节点管理
- 智能模式切换
- 自动初始位姿发布
- Web界面集成控制

## 系统架构

```
┌────────────────────────────────────────────────────────────────┐
│                        Web控制面板                               │
│                  (导航模式切换 + 地图保存)                        │
└───────────────────────────┬────────────────────────────────────┘
                            │ WebSocket
┌───────────────────────────┴────────────────────────────────────┐
│                   navigation_mode_manager                       │
│              (模式管理 + 生命周期控制 + 地图保存)                  │
└───┬─────────────┬──────────────┬─────────────┬─────────────────┘
    │             │              │             │
    │ 手动模式    │ 建图模式     │ 导航模式    │
    │             │              │             │
    v             v              v             v
┌─────┐    ┌────────────┐  ┌─────────┐  ┌──────────┐
│摇杆 │    │SLAM Toolbox│  │  AMCL   │  │  Nav2    │
│控制 │    │   (建图)   │  │ (定位)  │  │ (导航栈) │
└─────┘    └────────────┘  └─────────┘  └──────────┘
                │                 │           │
                v                 v           v
          ┌─────────────────────────────────────┐
          │           RPLidar A1                │
          │      (激光雷达 + /scan话题)         │
          └─────────────────────────────────────┘
                          │
                          v
          ┌─────────────────────────────────────┐
          │       EKF融合里程计                  │
          │    (/odometry/filtered)             │
          └─────────────────────────────────────┘
```

## 安装依赖

### 系统要求
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+

### 安装Nav2导航栈

```bash
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization
```

### 安装其他依赖

```bash
sudo apt install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-diagnostic-updater \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers
```

### Python依赖（如需要）

```bash
pip3 install numpy scipy
```

## 编译

```bash
cd ~/cleanbot_ws  # 或你的工作空间路径
colcon build --packages-select cleanbot_navigation
source install/setup.bash
```

## 使用方法

### 1. 完整系统启动

启动完整的机器人控制和导航系统：

```bash
# 终端1：启动机器人基础控制（USB通讯、EKF、控制器）
ros2 launch cleanbot_control robot_bringup.launch.py

# 终端2：启动Web控制面板
ros2 launch cleanbot_control web_control.launch.py

# 终端3：启动导航系统
ros2 launch cleanbot_navigation navigation_bringup.launch.py
```

### 2. 仅测试SLAM建图

```bash
# 确保机器人基础控制已启动
ros2 launch cleanbot_navigation slam_only.launch.py
```

### 3. 仅测试定位

```bash
# 需要指定已保存的地图
ros2 launch cleanbot_navigation localization_only.launch.py \
    map:=/path/to/your/map.yaml
```

## 操作流程

### 建图流程

1. **启动系统**
   - 按照上述"完整系统启动"方法启动所有节点
   
2. **打开Web界面**
   - 浏览器访问：`http://<机器人IP>:8080`
   - 确认WebSocket和USB连接状态正常
   
3. **切换到建图模式**
   - 在"导航模式"面板点击"建图"按钮
   - 系统会自动激活SLAM Toolbox节点
   - 此时摇杆仍然可用于手动控制
   
4. **手动驾驶建图**
   - 使用摇杆控制机器人在环境中移动
   - 观察地图逐渐生成（可用RViz2可视化）
   - 尽量覆盖所有需要的区域
   - 建议形成闭环路径以触发闭环检测
   
5. **保存地图**
   - 完成建图后，点击"保存地图"按钮
   - 地图将保存到：`~/cleanbot_maps/cleanbot_map_<时间戳>.yaml`
   - 同时生成对应的`.pgm`图像文件

### 导航流程

1. **确保有已保存的地图**
   - 地图文件位于：`~/cleanbot_maps/`
   
2. **修改启动文件指定地图**
   ```bash
   ros2 launch cleanbot_navigation navigation_bringup.launch.py \
       map:=~/cleanbot_maps/cleanbot_map_20251214_120000.yaml
   ```
   
3. **切换到导航模式**
   - 在Web界面点击"导航"按钮
   - 系统会自动：
     - 加载地图
     - 激活AMCL定位
     - 启动Nav2导航栈
     - 发布初始位姿（使用当前里程计位置）
     - **禁用摇杆控制**
   
4. **发送导航目标**
   - 使用RViz2的"2D Goal Pose"工具设置目标点
   - 或通过代码发布到`/goal_pose`话题
   - 机器人将自主导航到目标点

5. **监控导航状态**
   - 在RViz2中观察路径规划和机器人位置
   - Web界面查看系统日志
   - 紧急情况可切换回手动模式

## 配置文件说明

### `config/slam_toolbox.yaml`
SLAM建图参数配置：
- `resolution`: 地图分辨率（默认0.05m/像素）
- `minimum_travel_distance`: 最小移动距离触发更新（0.2m）
- `loop_search_maximum_distance`: 闭环搜索最大距离（3.0m）
- `do_loop_closing`: 启用闭环检测（true）

### `config/amcl.yaml`
AMCL定位参数配置：
- `min_particles`/`max_particles`: 粒子数量范围（500-2000）
- `update_min_d`/`update_min_a`: 更新阈值（0.15m, 0.2rad）
- `alpha1-5`: 里程计噪声模型参数（根据实际机器人调整）

### `config/nav2_params.yaml`
Nav2导航栈参数配置：
- `max_vel_x`/`max_vel_theta`: 最大速度限制
- `robot_radius`: 机器人半径（0.15m）
- `inflation_radius`: 障碍物膨胀半径（0.3m）
- 包含控制器、规划器、代价地图等完整配置

## 话题和服务

### 订阅的话题
- `/scan` (sensor_msgs/LaserScan): 激光雷达数据
- `/odometry/filtered` (nav_msgs/Odometry): EKF融合里程计

### 发布的话题
- `/navigation/mode_status` (std_msgs/UInt8): 当前导航模式
- `/navigation/info` (std_msgs/String): 导航系统信息
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): AMCL初始位姿

### 订阅的命令
- `/navigation/mode_cmd` (std_msgs/UInt8): 模式切换命令
  - 0: 手动模式
  - 1: 建图模式
  - 2: 导航模式

### 提供的服务
- `/navigation/save_map` (std_srvs/Empty): 保存当前地图

### 生命周期服务
- `/slam_toolbox/change_state`
- `/amcl/change_state`
- `/map_server/change_state`

## 可视化（RViz2）

启动RViz2进行可视化：

```bash
ros2 run rviz2 rviz2
```

推荐配置：
- Fixed Frame: `map`（导航模式）或 `odom`（建图模式）
- 添加显示：
  - LaserScan (`/scan`)
  - Map (`/map`)
  - RobotModel
  - Path (`/plan`)
  - PoseArray (`/particlecloud` - AMCL粒子)
  - TF

## 参数调优建议

### 建图质量优化
1. **降低建图速度**：慢速移动可提高建图质量
2. **闭环检测**：确保机器人经过之前区域形成闭环
3. **分辨率选择**：小环境用0.05m，大环境可用0.1m

### 定位精度优化
1. **粒子数量**：环境复杂度高时增加粒子数
2. **更新阈值**：减小`update_min_d/a`可提高定位频率但增加计算量
3. **初始位姿**：确保建图和导航时的起点大致相同

### 导航性能优化
1. **速度限制**：根据机器人硬件能力调整`max_vel_x`
2. **避障距离**：调整`inflation_radius`平衡安全性和通过性
3. **路径平滑**：启用smoother_server可获得更平滑的路径

## 故障排除

### 1. SLAM节点无法激活
**症状**：切换到建图模式后无响应

**解决方案**：
```bash
# 检查SLAM Toolbox是否运行
ros2 node list | grep slam

# 检查生命周期状态
ros2 lifecycle get /slam_toolbox

# 手动激活
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

### 2. 地图保存失败
**症状**：点击保存地图按钮无反应或报错

**解决方案**：
```bash
# 确保地图目录存在且有写权限
mkdir -p ~/cleanbot_maps
chmod 755 ~/cleanbot_maps

# 检查map_saver服务
ros2 service list | grep map

# 手动保存地图
ros2 run nav2_map_server map_saver_cli -f ~/cleanbot_maps/test_map
```

### 3. AMCL定位不准确
**症状**：机器人在地图上的位置漂移

**解决方案**：
- 确保建图和导航的起点大致相同
- 检查里程计数据是否正常：`ros2 topic echo /odometry/filtered`
- 增加粒子数量（修改`amcl.yaml`）
- 使用RViz2的"2D Pose Estimate"工具手动校正初始位姿

### 4. 导航模式下摇杆仍可用
**症状**：导航模式下摇杆控制没有被禁用

**解决方案**：
- 刷新Web页面
- 检查JavaScript控制台是否有错误
- 确认WebSocket连接正常

### 5. 激光雷达数据缺失
**症状**：`/scan`话题无数据

**解决方案**：
```bash
# 检查雷达设备
ls /dev/ttyUSB*

# 检查雷达节点
ros2 node list | grep rplidar

# 重启雷达节点
ros2 launch cleanbot_navigation navigation_bringup.launch.py
```

## 性能指标

### 建图性能
- 地图更新频率：~0.5 Hz
- 闭环检测延迟：< 1s
- 支持环境大小：100m × 100m

### 定位性能
- 定位更新频率：~10 Hz
- 定位精度：±0.1m (位置), ±5° (角度)
- 粒子收敛时间：< 5s

### 导航性能
- 规划频率：1-20 Hz
- 控制频率：20 Hz
- 动态避障反应时间：< 100ms

## 扩展功能

### 多楼层地图
修改启动文件支持多地图切换：
```python
# 添加地图选择参数
DeclareLaunchArgument('floor', default_value='1')
map_yaml_file = f'~/cleanbot_maps/floor_{floor}.yaml'
```

### 自动初始化定位
在导航模式启动时自动多点初始化：
```python
# 在navigation_mode_manager.py中
def publish_multiple_initial_poses(self):
    # 发布多个候选初始位姿
    for pose in candidate_poses:
        self.initialpose_pub.publish(pose)
```

### 巡检任务
使用waypoint_follower实现固定路线巡检：
```bash
ros2 action send_goal /follow_waypoints \
    nav2_msgs/action/FollowWaypoints \
    "{poses: [{...}, {...}]}"
```

## 相关资源

- [Nav2官方文档](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [AMCL参数说明](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [ROS 2 Humble文档](https://docs.ros.org/en/humble/)

## 维护和支持

- 作者：CleanBot Team
- 邮箱：3210676508@qq.com
- 更新日期：2025-12-14

## 许可证

Apache-2.0 License


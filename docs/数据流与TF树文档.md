# CleanBot 数据流与TF树文档

## 概述
本文档详细说明了CleanBot系统的数据流、话题订阅关系和TF树结构，涵盖实机和Gazebo仿真两种模式。

---

## 1. 系统架构对比

### 1.1 实机模式 (cleanbot.launch.py)

```
┌─────────────────────────────────────────────────────────────┐
│                    实机控制系统架构                           │
└─────────────────────────────────────────────────────────────┘

STM32硬件
    ↕ USB串口
USB通讯节点 (usb_communication_node.py)
    ↓ 发布: imu/data_raw, joint_states
    ↑ 订阅: wheel_speed_cmd, control_command
    
Hardware Interface (cleanbot_hardware_interface.cpp)
    ↓ 命令接口
    ↑ 状态接口
    
ros2_control (controller_manager)
    ├── diff_drive_controller
    │   ↓ 发布: /odom (禁用)
    │   ↑ 订阅: /diff_drive_controller/cmd_vel
    └── joint_state_broadcaster
        ↓ 发布: /joint_states (合并)

Robot State Publisher
    ↓ 发布: TF树 (base_link -> wheels, sensors)

EKF (robot_localization)
    ↓ 发布: odometry/filtered, TF (odom -> base_footprint)
    ↑ 订阅: imu/data_raw, joint_states

Web控制节点 (web_control_node.py)
    ↓ 发布: /diff_drive_controller/cmd_vel (TwistStamped)
    ↑ 订阅: imu/data_raw, joint_states, odometry/filtered, usb_connected, etc.
    ↕ WebSocket客户端通讯

RPLidar节点 (可选，独立启动)
    ↓ 发布: /scan
```

### 1.2 仿真模式 (cleanbot_gazebo.launch.py)

```
┌─────────────────────────────────────────────────────────────┐
│                   Gazebo仿真系统架构                          │
└─────────────────────────────────────────────────────────────┘

Gazebo物理仿真引擎
    ├── 激光雷达插件
    │   ↓ 发布: /scan
    ├── IMU插件
    │   ↓ 发布: imu/data_raw
    └── gazebo_ros2_control插件
        ↕ 与ros2_control通讯

ros2_control (controller_manager)
    ├── diff_drive_controller
    │   ↓ 发布: /odom (禁用)
    │   ↑ 订阅: /diff_drive_controller/cmd_vel
    └── joint_state_broadcaster
        ↓ 发布: /joint_states

Robot State Publisher
    ↓ 发布: TF树 (base_link -> wheels, sensors)

EKF (robot_localization)
    ↓ 发布: odometry/filtered, TF (odom -> base_footprint)
    ↑ 订阅: imu/data_raw, joint_states

Web控制节点 (web_control_node.py)
    ↓ 发布: /diff_drive_controller/cmd_vel (TwistStamped)
    ↑ 订阅: imu/data_raw, joint_states, odometry/filtered, etc.
    ↕ WebSocket客户端通讯
```

---

## 2. 话题数据流详解

### 2.1 控制命令流

```
Web界面/导航系统
    ↓ TwistStamped
/diff_drive_controller/cmd_vel
    ↓
diff_drive_controller
    ↓ 速度转换为轮速
left_wheel_joint/velocity/command, right_wheel_joint/velocity/command
    ↓
【实机】Hardware Interface → USB节点 → STM32
【仿真】Gazebo ros2_control → Gazebo物理引擎
```

### 2.2 传感器数据流

#### IMU数据流
```
【实机】STM32 → USB节点 → imu/data_raw (sensor_msgs/Imu)
【仿真】Gazebo IMU插件 → imu/data_raw (sensor_msgs/Imu)
    ↓
EKF节点 (融合)
    ↓
odometry/filtered (nav_msgs/Odometry)
```

#### 关节状态数据流
```
【实机】STM32 → USB节点 → joint_states (sensor_msgs/JointState)
【仿真】Gazebo → joint_state_broadcaster → joint_states
    ↓
EKF节点 (里程计融合)
Robot State Publisher (TF发布)
```

#### 激光雷达数据流
```
【实机】RPLidar硬件 → rplidar_node → /scan (sensor_msgs/LaserScan)
【仿真】Gazebo 激光雷达插件 → /scan (sensor_msgs/LaserScan)
    ↓
导航系统 (SLAM/定位/避障)
```

---

## 3. TF树结构

### 3.1 完整TF树
```
map (由SLAM/定位提供)
  └── odom (由EKF发布)
       └── base_footprint (地面投影)
            └── base_link (机器人中心)
                 ├── left_wheel_link (左轮)
                 ├── right_wheel_link (右轮)
                 ├── front_caster_link (前万向轮, 仅仿真)
                 ├── rear_caster_link (后万向轮, 仅仿真)
                 ├── imu_link (IMU传感器)
                 └── laser_support_link (雷达支撑杆)
                      └── laser (激光雷达)
```

### 3.2 TF发布者

| TF变换 | 发布者 | 频率 | 说明 |
|--------|--------|------|------|
| map → odom | SLAM/AMCL | 可变 | 全局定位 |
| odom → base_footprint | EKF | 50Hz | 融合里程计 |
| base_footprint → base_link | robot_state_publisher | 50Hz | 固定 (z=0.06) |
| base_link → wheels | robot_state_publisher | 50Hz | 关节变换 |
| base_link → sensors | robot_state_publisher | 50Hz | 传感器固定安装 |

---

## 4. 核心话题列表

### 4.1 控制相关

| 话题名 | 类型 | 发布者 | 订阅者 | 说明 |
|--------|------|--------|--------|------|
| /diff_drive_controller/cmd_vel | TwistStamped | web_control_node / nav2 | diff_drive_controller | 机器人速度命令 |
| wheel_speed_cmd | Float32MultiArray | hardware_interface | usb_communication_node (仅实机) | 轮速命令 (m/s) |

### 4.2 传感器相关

| 话题名 | 类型 | 发布者 | 订阅者 | 说明 |
|--------|------|--------|--------|------|
| imu/data_raw | Imu | usb_node (实机) / gazebo (仿真) | ekf_node, web_control | IMU原始数据 |
| joint_states | JointState | usb_node (实机) / joint_state_broadcaster (仿真) | ekf_node, robot_state_publisher | 关节状态 |
| /scan | LaserScan | rplidar_node (实机) / gazebo (仿真) | nav2, rviz | 激光雷达扫描 |

### 4.3 里程计相关

| 话题名 | 类型 | 发布者 | 订阅者 | 说明 |
|--------|------|--------|--------|------|
| odometry/filtered | Odometry | ekf_node | nav2, web_control | 融合后的里程计 |

### 4.4 状态监控 (仅实机)

| 话题名 | 类型 | 发布者 | 订阅者 | 说明 |
|--------|------|--------|--------|------|
| usb_connected | Bool | usb_communication_node | web_control_node | USB连接状态 |
| comm_quality | String | usb_communication_node | web_control_node | 通讯质量 |
| sensors_status | UInt8MultiArray | usb_communication_node | web_control_node | 传感器状态 |

---

## 5. 参数配置

### 5.1 机器人物理参数

| 参数 | 值 | 说明 |
|------|-----|------|
| wheel_radius | 0.032 m | 轮子半径 |
| wheel_separation | 0.2 m | 轮距 |
| max_linear_velocity | 0.5 m/s | 最大线速度 |
| max_angular_velocity | 2.0 rad/s | 最大角速度 |

### 5.2 控制器参数

| 参数 | 值 | 说明 |
|------|-----|------|
| update_rate | 50 Hz | 控制器更新频率 |
| use_sim_time | false (实机) / true (仿真) | 使用仿真时间 |
| open_loop | true | 开环控制 |
| position_feedback | false | 不使用位置反馈 |
| enable_odom_tf | false | 不发布odom TF (由EKF发布) |

### 5.3 EKF参数

| 参数 | 值 | 说明 |
|------|-----|------|
| frequency | 50 Hz | EKF更新频率 |
| odom_frame | odom | 里程计坐标系 |
| base_link_frame | base_footprint | 机器人坐标系 |
| world_frame | odom | 世界坐标系 |

---

## 6. 数据流完整性检查

### 6.1 检查命令

```bash
# 检查所有活跃话题
ros2 topic list

# 检查TF树
ros2 run tf2_tools view_frames

# 检查话题频率
ros2 topic hz /diff_drive_controller/cmd_vel
ros2 topic hz imu/data_raw
ros2 topic hz joint_states
ros2 topic hz /scan
ros2 topic hz odometry/filtered

# 检查TF变换
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link laser
```

### 6.2 预期输出

**实机模式：**
- imu/data_raw: ~50-100 Hz
- joint_states: ~50 Hz
- /scan: ~10 Hz (RPLidar A1)
- odometry/filtered: ~50 Hz
- TF树完整无断链

**仿真模式：**
- imu/data_raw: ~100 Hz
- joint_states: ~50 Hz
- /scan: ~10 Hz
- odometry/filtered: ~50 Hz
- TF树完整无断链

---

## 7. 故障排查

### 7.1 常见问题

**问题1：TF树断链**
- 检查：`ros2 run tf2_tools view_frames`
- 原因：某个TF发布节点未启动
- 解决：按launch文件中的延迟顺序检查各节点状态

**问题2：机器人不响应命令**
- 检查：`ros2 topic echo /diff_drive_controller/cmd_vel`
- 原因：控制器未启动或话题名错误
- 解决：确认controller_manager和spawner正常运行

**问题3：IMU数据异常**
- 实机：检查USB连接和通讯质量
- 仿真：检查Gazebo插件配置

**问题4：定位漂移**
- 检查：EKF参数配置
- 原因：传感器噪声或协方差矩阵不合理
- 解决：调整ekf.yaml中的协方差参数

---

## 8. 实机与仿真差异总结

| 项目 | 实机 | 仿真 |
|------|------|------|
| USB通讯节点 | ✓ 需要 | ✗ 不需要 |
| Hardware Interface | ✓ 自定义 | ✗ 使用GazeboSystem |
| IMU数据源 | STM32通过USB | Gazebo IMU插件 |
| 关节状态源 | STM32通过USB | Gazebo joint_state_broadcaster |
| 激光雷达 | RPLidar节点 | Gazebo ray sensor |
| 物理引擎 | 真实世界 | Gazebo ODE |
| use_sim_time | false | true |
| 万向轮 | 无显式模型 | 有显式模型 |

---

## 9. 调试建议

### 9.1 可视化工具

```bash
# 启动RViz2查看TF和传感器数据
ros2 run rviz2 rviz2

# 使用rqt_graph查看节点连接
rqt_graph

# 使用rqt_plot绘制数据曲线
rqt_plot
```

### 9.2 日志记录

```bash
# 记录所有话题数据
ros2 bag record -a

# 记录特定话题
ros2 bag record /scan imu/data_raw joint_states odometry/filtered

# 回放数据
ros2 bag play <bag_file>
```

---

## 10. 性能指标

### 10.1 实时性要求

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 控制循环延迟 | < 20ms | cmd_vel到轮速命令 |
| IMU数据延迟 | < 10ms | 传感器到ROS |
| 雷达扫描延迟 | < 100ms | 完整一帧扫描 |
| EKF更新延迟 | < 20ms | 融合计算 |

### 10.2 数据同步

- 所有传感器数据带时间戳
- EKF使用消息时间戳进行数据关联
- 仿真模式使用统一的仿真时钟

---

**文档版本**: 1.0  
**最后更新**: 2025-12-20  
**维护者**: CleanBot开发团队


















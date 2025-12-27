# 里程模式PID控制器详解

## 概述

里程模式下有**两个独立的PID闭环控制器**：
1. **路程控制器（Distance PID）** - 控制机器人移动的距离
2. **航向控制器（Yaw PID）** - 控制机器人旋转的角度

---

## 一、路程控制器（Distance PID）

### 1.1 输入数据

| 数据名称 | 变量名 | 类型 | 单位 | 数据来源 |
|---------|--------|------|------|---------|
| **目标路程** | `self.target_distance` | float | **米 (m)** | 前端输入的增量 + 当前累积路程 |
| **当前路程** | `self.accumulated_distance` | float | **米 (m)** | 基于里程计实时计算 |
| **当前时间** | `current_time` | float | **秒 (s)** | ROS时钟（支持仿真/实机时间） |

#### 数据来源详解

**目标路程**：
```python
# 来自前端命令：manual_control_cmd
# 格式: [control_mode, linear_vel, angular_vel, target_distance, target_yaw]
distance_increment = float(msg.data[3])  # 增量 (m)

# 计算目标路程 = 当前累积路程 + 增量
self.target_distance = self.accumulated_distance + distance_increment
```

**当前路程**：
```python
# 订阅话题：/odometry/filtered (来自EKF融合节点)
# 消息类型：nav_msgs/Odometry

def odom_callback(self, msg: Odometry):
    current_x = msg.pose.pose.position.x  # 当前X位置 (m)
    current_y = msg.pose.pose.position.y  # 当前Y位置 (m)
    
    # 计算增量距离（欧几里得距离）
    dx = current_x - self.last_odom_x
    dy = current_y - self.last_odom_y
    delta_distance = math.sqrt(dx**2 + dy**2)  # 米 (m)
    
    # 累加到总路程
    self.accumulated_distance += delta_distance
```

**当前时间**：
```python
current_time = self.get_clock().now().nanoseconds / 1e9  # 秒 (s)
# 注意：支持仿真时间和实机时间，根据use_sim_time参数自动切换
```

### 1.2 PID计算过程

```python
# 调用PID控制器
vx = self.distance_pid.compute(
    setpoint=self.target_distance,      # 目标值 (m)
    measurement=self.accumulated_distance,  # 测量值 (m)
    current_time=current_time            # 当前时间 (s)
)
```

#### 误差计算
```python
error = setpoint - measurement  # 单位: 米 (m)
# error = 目标路程 - 当前路程
# 正值：需要前进
# 负值：需要后退
```

#### PID三项计算

**比例项 (P)**：
```python
p_term = kp * error  # 单位: m/s
# kp = 0.8 (默认值，可配置)
# 含义：误差越大，输出速度越大
```

**积分项 (I)**：
```python
integral += error * dt  # 累积误差 (m·s)
i_term = ki * integral  # 单位: m/s
# ki = 0.01 (默认值)
# 含义：消除稳态误差
```

**微分项 (D)**：
```python
derivative = (error - last_error) / dt  # 误差变化率 (m/s)
d_term = kd * derivative  # 单位: m/s
# kd = 0.15 (默认值)
# 含义：抑制振荡，提前减速
```

**总输出**：
```python
output = p_term + i_term + d_term  # 单位: m/s
# 限幅：-0.3 ~ 0.3 m/s (默认output_limit)
```

### 1.3 输出数据

| 数据名称 | 变量名 | 类型 | 单位 | 输出范围 |
|---------|--------|------|------|---------|
| **线速度** | `vx` | float | **米/秒 (m/s)** | -0.3 ~ 0.3 (可配置) |

**输出方向**：
- `vx > 0`：机器人前进
- `vx < 0`：机器人后退
- `vx = 0`：停止

---

## 二、航向控制器（Yaw PID）

### 2.1 输入数据

| 数据名称 | 变量名 | 类型 | 单位 | 数据来源 |
|---------|--------|------|------|---------|
| **目标航向** | `self.target_yaw` | float | **弧度 (rad)** | 前端输入的增量 + 当前航向 |
| **当前航向** | `self.current_yaw` | float | **弧度 (rad)** | IMU四元数解算 |
| **当前时间** | `current_time` | float | **秒 (s)** | ROS时钟 |

#### 数据来源详解

**目标航向**：
```python
# 来自前端命令：manual_control_cmd
# 格式: [control_mode, linear_vel, angular_vel, target_distance, target_yaw]
yaw_increment = float(msg.data[4])  # 增量 (rad)

# 计算目标航向 = 当前航向 + 增量，并归一化到[-π, π]
self.target_yaw = self.normalize_angle(self.current_yaw + yaw_increment)
```

**当前航向**：
```python
# 订阅话题：/imu/data_raw (来自IMU传感器或Gazebo)
# 消息类型：sensor_msgs/Imu

def imu_callback(self, msg: Imu):
    # 从四元数提取yaw角
    q = msg.orientation  # 四元数 (x, y, z, w)
    
    # 四元数转欧拉角（Yaw）
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    self.current_yaw = math.atan2(siny_cosp, cosy_cosp)  # 弧度 (rad)
    # 范围：-π ~ π (-3.14 ~ 3.14)
```

### 2.2 PID计算过程

```python
# 计算角度误差（处理周期性）
yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)

# 调用PID控制器（注意：setpoint=0，measurement=-error）
wz = self.yaw_pid.compute(
    setpoint=0.0,           # 目标值设为0
    measurement=-yaw_error, # 测量值设为负误差
    current_time=current_time
)
```

#### 角度归一化
```python
def normalize_angle(angle):
    """将角度归一化到[-π, π]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
```

**为什么需要归一化？**
- 角度是周期性的：359° = -1°
- 例如：从350°转到10°，直接相减是-340°，但实际应该转20°
- 归一化后确保走最短路径

#### 误差计算
```python
error = 0.0 - (-yaw_error) = yaw_error  # 单位: 弧度 (rad)
# yaw_error = 目标航向 - 当前航向
# 正值：需要左转（逆时针）
# 负值：需要右转（顺时针）
```

#### PID三项计算

**比例项 (P)**：
```python
p_term = kp * error  # 单位: rad/s
# kp = 1.2 (默认值)
# 含义：角度误差越大，转速越快
```

**积分项 (I)**：
```python
integral += error * dt
i_term = ki * integral  # 单位: rad/s
# ki = 0.0 (默认值，一般不用)
# 含义：角度控制通常不需要积分项
```

**微分项 (D)**：
```python
derivative = (error - last_error) / dt  # rad/s
d_term = kd * derivative  # 单位: rad/s
# kd = 0.25 (默认值)
# 含义：抑制转向过冲，平滑转向
```

**总输出**：
```python
output = p_term + i_term + d_term  # 单位: rad/s
# 限幅：-1.0 ~ 1.0 rad/s (默认output_limit)
```

### 2.3 输出数据

| 数据名称 | 变量名 | 类型 | 单位 | 输出范围 |
|---------|--------|------|------|---------|
| **角速度** | `wz` | float | **弧度/秒 (rad/s)** | -1.0 ~ 1.0 (可配置) |

**输出方向**：
- `wz > 0`：机器人左转（逆时针）
- `wz < 0`：机器人右转（顺时针）
- `wz = 0`：不转向

---

## 三、里程计算详解

### 3.1 路程定义

**路程（Distance）vs 位移（Displacement）**：
- **路程**：机器人实际行走的轨迹长度（标量）
- **位移**：起点到终点的直线距离（矢量）

**本系统使用路程**：
- 累积所有移动的距离，无论方向
- 即使机器人转圈，路程也会增加

### 3.2 路程计算方法

```python
def odom_callback(self, msg: Odometry):
    """里程计回调 - 解算累积路程"""
    # 获取当前位置（世界坐标系）
    current_x = msg.pose.pose.position.x  # 米 (m)
    current_y = msg.pose.pose.position.y  # 米 (m)
    
    if not self.odom_initialized:
        # 第一次初始化
        self.last_odom_x = current_x
        self.last_odom_y = current_y
        self.odom_initialized = True
        return
    
    # 计算本次移动的增量距离
    dx = current_x - self.last_odom_x  # X方向位移 (m)
    dy = current_y - self.last_odom_y  # Y方向位移 (m)
    
    # 欧几里得距离（勾股定理）
    delta_distance = math.sqrt(dx**2 + dy**2)  # 米 (m)
    
    # 累加到总路程
    self.accumulated_distance += delta_distance
    
    # 更新上次位置，准备下次计算
    self.last_odom_x = current_x
    self.last_odom_y = current_y
```

### 3.3 数据流图

```
EKF融合节点 (/robot_localization/ekf_node)
    ↓ 输出
/odometry/filtered (nav_msgs/Odometry)
    ├─ pose.pose.position.x (m)
    └─ pose.pose.position.y (m)
    ↓ 订阅
manual_control_node.odom_callback()
    ↓ 计算
delta_distance = √(dx² + dy²)
    ↓ 累加
accumulated_distance += delta_distance
    ↓ 用于
Distance PID Controller
```

### 3.4 路程计算示例

**示例1：直线前进**
```
起点: (0, 0)
移动1: (1, 0) → delta = 1.0m
移动2: (2, 0) → delta = 1.0m
移动3: (3, 0) → delta = 1.0m
累积路程 = 1.0 + 1.0 + 1.0 = 3.0m
```

**示例2：正方形轨迹**
```
起点: (0, 0)
移动1: (1, 0) → delta = 1.0m
移动2: (1, 1) → delta = 1.0m
移动3: (0, 1) → delta = 1.0m
移动4: (0, 0) → delta = 1.0m
累积路程 = 1.0 + 1.0 + 1.0 + 1.0 = 4.0m
（注意：虽然回到原点，但路程是4.0m，不是0）
```

**示例3：曲线运动**
```
机器人沿着圆弧移动
每个小段都会被累加
最终路程 = 所有小段之和
```

---

## 四、控制循环流程

### 4.1 完整控制流程

```python
def control_loop(self):  # 50Hz (每20ms执行一次)
    # 1. 检查导航模式
    if self.navigation_mode == NAV_MODE_NAVIGATION:
        return  # 导航模式下不输出
    
    # 2. 计算速度命令
    if self.control_mode == CONTROL_MODE_ODOMETRY:
        vx, wz = self.compute_odometry_control()
    
    # 3. 红外安全融合
    vx, wz = self.apply_ir_safety(vx, wz)
    
    # 4. 发布速度命令
    self.publish_velocity(vx, wz)
```

### 4.2 里程控制函数

```python
def compute_odometry_control(self):
    """里程模式 - PID闭环控制"""
    # 0. 检查初始化
    if not self.odom_initialized or not self.imu_initialized:
        return 0.0, 0.0
    
    # 1. 获取当前时间
    current_time = self.get_clock().now().nanoseconds / 1e9  # 秒
    
    # 2. 路程PID控制
    vx = self.distance_pid.compute(
        setpoint=self.target_distance,          # 目标路程 (m)
        measurement=self.accumulated_distance,  # 当前路程 (m)
        current_time=current_time               # 时间 (s)
    )  # 输出: m/s
    
    # 3. 航向PID控制
    yaw_error = self.normalize_angle(
        self.target_yaw - self.current_yaw)  # 角度误差 (rad)
    
    wz = self.yaw_pid.compute(
        setpoint=0.0,              # 目标误差为0
        measurement=-yaw_error,    # 当前误差
        current_time=current_time  # 时间 (s)
    )  # 输出: rad/s
    
    # 4. 检查是否到达目标
    distance_error = abs(self.target_distance - self.accumulated_distance)
    yaw_error_abs = abs(yaw_error)
    
    if distance_error < self.distance_tolerance and \
       yaw_error_abs < self.yaw_tolerance:
        # 到达目标，停止
        vx, wz = 0.0, 0.0
        self.target_distance = self.accumulated_distance
        self.target_yaw = self.current_yaw
    
    # 5. 限幅
    vx = max(-self.max_linear_vel, min(self.max_linear_vel, vx))
    wz = max(-self.max_angular_vel, min(self.max_angular_vel, wz))
    
    return vx, wz
```

### 4.3 速度发布

```python
def publish_velocity(self, vx, wz):
    """发布速度命令 - TwistStamped类型"""
    cmd_msg = TwistStamped()
    
    # 时间戳（重要！支持仿真时间）
    cmd_msg.header.stamp = self.get_clock().now().to_msg()
    cmd_msg.header.frame_id = 'base_footprint'
    
    # 速度数据
    cmd_msg.twist.linear.x = vx   # m/s
    cmd_msg.twist.angular.z = wz  # rad/s
    
    # 发布到差速控制器
    self.cmd_vel_pub.publish(cmd_msg)
    # 话题: /diff_drive_controller/cmd_vel
```

---

## 五、参数配置

### 5.1 默认参数 (`config/manual_control_params.yaml`)

```yaml
manual_control_node:
  ros__parameters:
    control_frequency: 50.0  # 控制频率 (Hz)
    
    # 路程控制PID
    distance_pid:
      kp: 0.8              # 比例系数
      ki: 0.01             # 积分系数
      kd: 0.15             # 微分系数
      output_limit: 0.3    # 输出限幅 (m/s)
    
    # 航向控制PID
    yaw_pid:
      kp: 1.2              # 比例系数
      ki: 0.0              # 积分系数
      kd: 0.25             # 微分系数
      output_limit: 1.0    # 输出限幅 (rad/s)
    
    # 速度限制
    max_linear_velocity: 0.3   # 最大线速度 (m/s)
    max_angular_velocity: 1.0  # 最大角速度 (rad/s)
    
    # 控制容差
    distance_tolerance: 0.02   # 路程容差 (m)
    yaw_tolerance: 0.05        # 航向容差 (rad, 约2.86度)
```

### 5.2 参数调整建议

#### 路程控制调优
- **响应太慢**：增加 `kp`（例如 1.0）
- **振荡/超调**：增加 `kd`（例如 0.2），或减小 `kp`
- **稳态误差**：增加 `ki`（例如 0.02）
- **速度太快**：减小 `output_limit`

#### 航向控制调优
- **转向太慢**：增加 `kp`（例如 1.5）
- **转向过冲**：增加 `kd`（例如 0.3）
- **转向不稳**：减小 `kp`

---

## 六、数据流总览

```
┌─────────────────────────────────────────────────────────────┐
│                        前端Web界面                           │
│  用户输入: 距离=1.0m, 角度=90°                              │
└─────────────────────┬───────────────────────────────────────┘
                      │ WebSocket
                      ↓
┌─────────────────────────────────────────────────────────────┐
│                    web_control_node                          │
│  转换: 角度90° → 1.5708 rad                                 │
│  发布: manual_control_cmd [1, 0, 0, 1.0, 1.5708]           │
└─────────────────────┬───────────────────────────────────────┘
                      │ ROS话题
                      ↓
┌─────────────────────────────────────────────────────────────┐
│                 manual_control_node                          │
│                                                              │
│  ┌──────────────┐         ┌──────────────┐                 │
│  │ 里程计订阅    │         │   IMU订阅    │                 │
│  │ /odometry/   │         │  /imu/data   │                 │
│  │  filtered    │         │    _raw      │                 │
│  └──────┬───────┘         └──────┬───────┘                 │
│         │                         │                          │
│         ↓                         ↓                          │
│  累积路程计算              四元数→Yaw转换                   │
│  (每次Δd累加)              (atan2计算)                      │
│         │                         │                          │
│         ↓                         ↓                          │
│  ┌────────────────┐      ┌────────────────┐                │
│  │ Distance PID   │      │   Yaw PID      │                │
│  │ 输入:          │      │  输入:         │                │
│  │  目标=2.0m     │      │   目标=1.57rad │                │
│  │  当前=1.0m     │      │   当前=0rad    │                │
│  │ 输出: vx=0.8m/s│      │  输出: wz=1.2  │                │
│  └────────┬───────┘      └────────┬───────┘                │
│           │                       │                          │
│           └───────────┬───────────┘                          │
│                       ↓                                      │
│              红外安全融合                                    │
│                       ↓                                      │
│            发布 TwistStamped                                 │
│            {vx=0.8, wz=1.2}                                 │
└─────────────────────┬───────────────────────────────────────┘
                      │ /diff_drive_controller/cmd_vel
                      ↓
┌─────────────────────────────────────────────────────────────┐
│              ros2_control差速控制器                          │
│              (diff_drive_controller)                         │
└─────────────────────┬───────────────────────────────────────┘
                      │ 轮速命令
                      ↓
               机器人硬件/仿真
```

---

## 七、常见问题

### Q1: 为什么路程是累积的？
**A**: 路程表示机器人实际行走的总距离，不考虑方向。即使机器人转圈或后退，路程都会增加。这样可以准确控制机器人移动的总长度。

### Q2: 为什么航向PID的setpoint是0？
**A**: 这是一个技巧。我们先计算误差 `yaw_error`，然后让PID把误差控制到0，相当于让当前航向接近目标航向。

### Q3: 时间戳为什么重要？
**A**: 
- **仿真环境**：使用仿真时间，可以暂停、加速
- **实机环境**：使用系统时间
- TwistStamped类型的时间戳确保控制器知道命令的有效性

### Q4: 如何判断到达目标？
**A**: 同时满足两个条件：
- 路程误差 < 2cm (distance_tolerance)
- 航向误差 < 0.05rad ≈ 2.86° (yaw_tolerance)

### Q5: 红外触发时会怎样？
**A**: 
- 前方红外：立即修改目标路程为当前值，PID输出变为0
- 左/右红外：立即修改目标航向为当前值，停止转向
- 目的：避免碰撞，保护机器人

---

## 八、调试建议

### 查看实时数据
```bash
# 查看累积路程和航向
ros2 topic echo /distance_feedback

# 查看速度命令
ros2 topic echo /diff_drive_controller/cmd_vel

# 查看里程计
ros2 topic echo /odometry/filtered

# 查看IMU
ros2 topic echo /imu/data_raw
```

### 动态调参
```bash
# 修改路程PID的kp
ros2 param set /manual_control_node distance_pid.kp 1.0

# 修改航向PID的kp
ros2 param set /manual_control_node yaw_pid.kp 1.5
```

### 查看节点信息
```bash
ros2 node info /manual_control_node
```

---

## 总结

**两个PID控制器协同工作**：
1. **Distance PID**：控制 "走多远"
   - 输入：目标路程(m) vs 累积路程(m)
   - 输出：线速度 vx (m/s)

2. **Yaw PID**：控制 "转多少"
   - 输入：目标航向(rad) vs 当前航向(rad)
   - 输出：角速度 wz (rad/s)

**路程计算**：基于里程计的x、y位置，累积每次移动的欧几里得距离

**数据来源**：
- 里程计 → `/odometry/filtered` (EKF融合)
- IMU → `/imu/data_raw`
- 时间 → ROS Clock (支持仿真/实机)

**关键特性**：
- ✅ 独立控制距离和方向
- ✅ 支持仿真时间和实机时间
- ✅ 红外安全保护
- ✅ 参数可动态调整
- ✅ TwistStamped类型（带时间戳）







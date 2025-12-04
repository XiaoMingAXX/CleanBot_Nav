# CleanBot Control System - 快速启动指南

## 系统概述

CleanBot控制系统已成功构建，包含以下功能模块：

### ✅ 已实现的功能

1. **USB通讯节点** 
   - 完整的STM32通讯协议实现（CRC校验、帧解析、序列号管理）
   - 支持无串口启动，后台自动重连
   - 通讯质量监测（帧率、丢包率）
   - 支持Web界面动态扫描和连接串口

2. **ros2_control硬件接口**
   - 标准化硬件接口实现
   - 差速控制器配置
   - （需要安装ros2_control依赖）

3. **EKF定位节点配置**
   - 融合轮速和IMU数据的扩展卡尔曼滤波
   - 配置文件已就绪

4. **Web控制界面**
   - 实时状态监控（IMU、轮速、传感器、里程计）
   - 虚拟摇杆速度控制
   - 工作模式切换
   - 执行器档位控制
   - 通讯质量显示
   - 串口扫描与连接

## 快速启动步骤

### 1. 安装必要依赖（可选）

如果需要完整功能，请安装以下依赖：

```bash
# ros2_control相关（用于标准化硬件接口）
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-hardware-interface \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-robot-localization

# Python依赖（用于Web界面）
sudo apt install -y \
  python3-serial \
  python3-numpy \
  python3-aiohttp
```

### 2. 编译项目（如已编译可跳过）

```bash
cd /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws
colcon build --packages-select cleanbot_control --symlink-install
source install/setup.bash
```

### 3. 启动系统

**方式一：基础系统（推荐先用这个测试）**
```bash
source /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws/install/setup.bash
ros2 launch cleanbot_control cleanbot_basic.launch.py
```

**方式二：完整系统（需要安装ros2_control）**
```bash
source /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws/install/setup.bash
ros2 launch cleanbot_control cleanbot_control.launch.py
```

**自定义串口：**
```bash
ros2 launch cleanbot_control cleanbot_basic.launch.py port:=/dev/ttyACM0
```

### 4. 访问Web控制面板

在浏览器中打开：**http://localhost:8080**

## Web控制面板使用说明

### 连接串口
1. 点击"扫描串口"按钮
2. 从下拉列表选择检测到的串口
3. 点击"连接"按钮

### 速度控制
- 使用虚拟摇杆控制机器人移动
- 向上/下：前进/后退
- 向左/右：转向
- 松开自动回中并停止

### 工作模式
- 待机：停止所有运动
- 自动全屋：自动清扫模式
- 沿边：沿墙清扫
- 弓形：弓形路径清扫
- 遥控：手动控制
- 回充：自动回充

### 执行器控制
- 左/右边刷：0-3档
- 风机：0-3档
- 水泵：0-3档

### 监控面板
- **连接状态**：WebSocket和USB串口连接状态
- **通讯质量**：各消息帧的帧率和丢包率
- **传感器状态**：碰撞传感器、红外传感器、回充状态
- **IMU数据**：加速度和角速度
- **轮速数据**：左右轮位置和速度
- **里程计数据**：机器人位置和速度
- **系统日志**：实时操作日志

## 通讯协议说明

### 消息帧格式
```
[0x55][0xAA][VER][LEN_L][LEN_H][MSG_ID][SEQ][PAYLOAD...][CRC_L][CRC_H]
```

### 消息类型

**下行（树莓派→STM32）：**
- `0x10` CONTROL_CMD：控制命令（轮速、模式、档位）

**上行（STM32→树莓派）：**
- `0x20` IMU：IMU数据（100Hz）
- `0x21` WHEEL：轮速数据（200Hz）
- `0x22` SENSORS_STATUS：传感器状态（20-50Hz）
- `0x23` SYSTEM_STATUS：系统状态（1-5Hz）
- `0x24` ACK_REPLY：ACK回复

详细协议定义见：`STM32 端 USB 通讯协议实现任务文档.md`

## 故障排除

### 1. 串口连接失败
**现象**：日志显示"无法连接到/dev/ttyACM0"

**解决方案**：
- 检查STM32是否已连接：`ls /dev/ttyACM* /dev/ttyUSB*`
- 检查串口权限：`sudo usermod -a -G dialout $USER`（需要重新登录）
- 使用Web界面的"扫描串口"功能查找正确的端口
- 系统设计为无串口时也能启动，可以先测试Web界面

### 2. Web界面无法访问
**现象**：浏览器无法打开http://localhost:8080

**解决方案**：
- 检查端口是否被占用：`lsof -i :8080`
- 更换端口：`ros2 launch cleanbot_control cleanbot_basic.launch.py web_port:=8081`
- 检查Python依赖：`python3 -c "import aiohttp"`

### 3. ros2_control相关错误
**现象**：编译或运行时出现controller_manager错误

**解决方案**：
- 使用基础启动文件：`cleanbot_basic.launch.py`（不需要ros2_control）
- 或安装完整依赖（见上方安装步骤）

### 4. Python模块未找到
**现象**：`ModuleNotFoundError: No module named 'serial'` 等

**解决方案**：
```bash
sudo apt install python3-serial python3-numpy python3-aiohttp
```

## 测试建议

### 阶段1：基础功能测试（无硬件）
1. 启动基础系统：`ros2 launch cleanbot_control cleanbot_basic.launch.py`
2. 打开Web界面，确认WebSocket连接正常
3. 测试虚拟摇杆（即使没有串口，命令也会发布到ROS话题）
4. 查看系统日志面板

### 阶段2：串口通讯测试（连接STM32）
1. 连接STM32设备
2. 在Web界面扫描并连接串口
3. 观察通讯质量指标
4. 测试发送控制命令
5. 查看接收到的传感器数据

### 阶段3：完整系统测试
1. 安装ros2_control依赖
2. 重新编译
3. 启动完整系统：`ros2 launch cleanbot_control cleanbot_control.launch.py`
4. 测试导航和定位功能

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                   Web Browser (Port 8080)                   │
│                   - 控制面板                                 │
│                   - 实时监控                                 │
└─────────────────────┬───────────────────────────────────────┘
                      │ WebSocket + HTTP
┌─────────────────────┴───────────────────────────────────────┐
│         Web Control Node (web_control_node.py)              │
│  - WebSocket服务器                                          │
│  - 状态数据聚合与推送                                       │
│  - 控制命令转发                                             │
└─────────────────────┬───────────────────────────────────────┘
                      │ ROS2 Topics
        ┌─────────────┼─────────────────┐
        │             │                 │
┌───────┴──────┐  ┌──┴─────────┐  ┌───┴──────────┐
│USB Comm Node │  │ros2_control│  │robot_        │
│(Protocol     │  │(Optional)  │  │localization  │
│ Handler)     │  │            │  │(EKF Fusion)  │
└───────┬──────┘  └────────────┘  └──────────────┘
        │
    ┌───┴────┐
    │ STM32  │ USB/Serial
    │(实际   │
    │ 硬件)  │
    └────────┘
```

## ROS2话题列表

运行后可用的话题：
```bash
# 查看所有话题
ros2 topic list

# 主要话题：
/cmd_vel                    # 速度命令
/imu/data_raw              # IMU原始数据
/joint_states              # 关节状态
/sensors_status            # 传感器状态
/usb_connected             # USB连接状态
/comm_quality              # 通讯质量
/system_status             # 系统状态
/odometry/filtered         # 融合后的里程计（需要EKF）
```

## 下一步开发建议

1. **完善硬件接口**：安装ros2_control后测试标准化控制
2. **集成导航功能**：添加navigation2支持
3. **地图构建**：集成SLAM算法
4. **路径规划**：实现自动清扫路径规划
5. **性能优化**：调整通讯频率和EKF参数

## 技术支持

- 详细协议文档：`STM32 端 USB 通讯协议实现任务文档.md`
- 系统说明：`src/cleanbot_control/README.md`
- 测试脚本：`test_cleanbot.sh`

---

**系统状态**：✅ 已编译并可运行
**核心功能**：✅ USB通讯、Web界面已完成
**可选功能**：⚠️ ros2_control需要安装依赖

祝你测试顺利！ 🤖


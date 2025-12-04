# CleanBot Control System

基于ROS2 Jazzy的扫地机器人导航控制系统。

## 功能模块

1. **USB通讯节点** - 实现STM32通讯协议的数据收发和质量监测
2. **ros2_control硬件接口** - 标准化硬件接口和差速控制器
3. **EKF定位节点** - 融合轮速和IMU数据的扩展卡尔曼滤波定位
4. **Web控制界面** - WebSocket后端服务器和网页前端控制面板

## 系统依赖

### 必需的ROS2包
```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-hardware-interface \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-robot-localization \
  ros-jazzy-robot-state-publisher
```

### Python依赖
```bash
pip3 install -r requirements.txt
```

或者使用系统包管理器：
```bash
sudo apt install -y \
  python3-serial \
  python3-numpy \
  python3-aiohttp
```

## 编译

```bash
cd /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws
colcon build --packages-select cleanbot_control --symlink-install
source install/setup.bash
```

## 运行

### 完整系统启动
```bash
ros2 launch cleanbot_control cleanbot_control.launch.py
```

### 自定义参数
```bash
ros2 launch cleanbot_control cleanbot_control.launch.py \
  port:=/dev/ttyACM0 \
  baudrate:=921600 \
  web_port:=8080
```

### 单独启动USB通讯节点
```bash
ros2 run cleanbot_control usb_communication_node.py \
  --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baudrate:=921600
```

### 单独启动Web控制界面
```bash
ros2 run cleanbot_control web_control_node.py \
  --ros-args \
  -p web_port:=8080
```

然后在浏览器中访问：http://localhost:8080

## 串口权限

如果遇到串口权限问题：
```bash
sudo usermod -a -G dialout $USER
# 重新登录或重启后生效
```

## 通讯协议

详见 `STM32 端 USB 通讯协议实现任务文档.md`

### 下行消息（树莓派→STM32）
- `0x10` CONTROL_CMD - 控制命令（轮速、模式、档位）

### 上行消息（STM32→树莓派）
- `0x20` IMU - IMU数据（100Hz）
- `0x21` WHEEL - 轮速数据（200Hz）
- `0x22` SENSORS_STATUS - 传感器状态（20-50Hz）
- `0x23` SYSTEM_STATUS - 系统状态（1-5Hz）
- `0x24` ACK_REPLY - ACK回复

## Web控制面板功能

- 实时机器人状态监控
- 虚拟摇杆速度控制
- 工作模式切换（待机/自动/沿边/弓形/遥控/回充）
- 执行器档位控制（边刷/风机/水泵）
- 传感器状态显示
- 串口扫描与连接
- 通讯质量监测
- 系统日志查看

## 故障排除

### 1. 串口无法连接
- 检查串口设备是否存在：`ls /dev/ttyACM*` 或 `ls /dev/ttyUSB*`
- 检查串口权限：确保用户在`dialout`组中
- 在Web界面使用"扫描串口"功能查找可用端口

### 2. WebSocket连接失败
- 确保端口8080未被占用
- 检查防火墙设置
- 查看节点日志：`ros2 node info /web_control_node`

### 3. EKF定位不工作
- 确保IMU和轮速数据正常发布
- 检查话题：`ros2 topic list`
- 查看EKF状态：`ros2 topic echo /diagnostics`

### 4. ros2_control无法加载
- 确保安装了所有必需的控制器包
- 检查硬件接口插件是否正确导出
- 查看controller_manager日志

## 配置文件

- `config/cleanbot_controllers.yaml` - ros2_control控制器配置
- `config/ekf.yaml` - EKF定位参数配置
- `config/cleanbot.ros2_control.xacro` - 硬件接口描述

## 架构说明

```
┌─────────────────────────────────────────────────────────────┐
│                      Web Frontend                           │
│                    (HTML + JavaScript)                      │
└─────────────────────┬───────────────────────────────────────┘
                      │ WebSocket
┌─────────────────────┴───────────────────────────────────────┐
│              Web Control Node (Python)                      │
│  - WebSocket服务器  - HTTP静态文件服务  - 状态推送         │
└─────────────────────┬───────────────────────────────────────┘
                      │ ROS2 Topics
        ┌─────────────┼─────────────┐
        │             │             │
┌───────┴────┐  ┌────┴────┐  ┌────┴─────────┐
│USB Comm    │  │ros2_    │  │robot_        │
│Node        │  │control  │  │localization  │
│(Protocol)  │  │(硬件接口)│  │(EKF融合)     │
└───────┬────┘  └────┬────┘  └────┬─────────┘
        │            │             │
    ┌───┴────────────┴─────────────┴───┐
    │      ROS2 Topic Infrastructure    │
    └───────────────────────────────────┘
```

## 开发者

- 维护者：xiaoming
- License: Apache-2.0

## 更新日志

### v0.0.1 (初始版本)
- 实现USB通讯协议
- 实现ros2_control硬件接口
- 实现EKF定位
- 实现Web控制界面
- 支持无串口启动



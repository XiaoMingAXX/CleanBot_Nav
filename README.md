# CleanBot ROS2 控制系统

基于 ROS2 的清洁机器人控制系统，采用 diff_drive_controller 实现差速底盘控制。

## 快速开始

### 1. 编译系统

```bash
cd /home/xiaoming/桌面/MOON/Electronic/CleanBot_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. 启动系统

```bash
ros2 launch cleanbot_control cleanbot.launch.py
```

### 3. 访问控制界面

打开浏览器访问：`http://localhost:8080`

## 系统架构

### 核心节点

1. **ros2_control_node** - 硬件接口和控制器管理
   - `cleanbot_hardware_interface` - 硬件接口实现
   - `diff_drive_controller` - 差速驱动控制器
   - `joint_state_broadcaster` - 关节状态发布器

2. **usb_communication_node** - USB 串口通信
   - 与 STM32 单片机通信
   - 发送轮速命令（m/s）
   - 接收轮速、IMU 等传感器数据

3. **web_control_node** - Web 控制界面
   - WebSocket 实时通信
   - 摇杆控制
   - 状态监控

4. **rplidar_node** - 激光雷达驱动（可选）

### 数据流

```
Web界面 → TwistStamped → diff_drive_controller → 关节速度(rad/s)
                                                        ↓
                                          硬件接口 → 轮速(m/s)
                                                        ↓
                                          USB节点 → STM32
                                                        ↓
                                          传感器反馈 ← STM32
```

## 配置文件

- `src/cleanbot_control/config/cleanbot_controllers.yaml` - 控制器参数
- `src/cleanbot_control/urdf/robot.urdf.xacro` - 机器人模型
- `src/cleanbot_control/launch/cleanbot.launch.py` - 启动配置

## 重要参数

### 控制器参数

- `wheel_radius: 0.032` - 轮子半径（米）
- `wheel_separation: 0.2` - 轮间距（米）
- `open_loop: true` - 开环控制模式
- `position_feedback: false` - 不使用位置反馈
- `cmd_vel_timeout: 10.0` - 命令超时（秒）

### 速度限制

- 线速度：最大 0.5 m/s，加速度 0.8 m/s²
- 角速度：最大 2.0 rad/s，加速度 1.5 rad/s²

## 文档

详细文档请查看 `docs/` 目录：

- `启动指南.md` - 系统启动和基本操作
- `QUICK_START.md` - 快速入门指南
- `STM32 端 USB 通讯协议实现任务文档.md` - 串口通信协议
- `控制架构说明.md` - 详细的控制架构说明
- `系统交付清单.md` - 系统交付内容清单
- `最终交付说明.md` - 最终交付说明

## 故障排查

### 机器人不动

1. 检查 USB 连接：在 Web 界面查看 USB 状态
2. 检查控制器状态：`ros2 control list_controllers`
3. 检查话题数据：`ros2 topic echo /wheel_speed_cmd`

### Web 界面无法连接

1. 确认系统已启动
2. 检查端口是否被占用
3. 清除浏览器缓存并刷新

### 控制器未激活

```bash
ros2 control set_controller_state diff_drive_controller active
```

## 开发者信息

- ROS2 版本：Humble/Foxy/Galactic
- Python 版本：3.8+
- 构建系统：colcon

## 许可证

请参考项目许可证文件。







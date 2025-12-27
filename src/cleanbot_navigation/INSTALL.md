# CleanBot Navigation 安装依赖清单

## 必需安装的ROS 2包

以下是所有需要安装的依赖包命令汇总。

### Nav2导航栈完整安装

```bash
sudo apt update

sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-smoother \
    ros-humble-nav2-velocity-smoother
```

### SLAM工具包

```bash
sudo apt install -y \
    ros-humble-slam-toolbox
```

### 定位和传感器融合

```bash
sudo apt install -y \
    ros-humble-robot-localization
```

### 其他必需组件

```bash
sudo apt install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-diagnostic-updater \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster
```

### 可视化和调试工具

```bash
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-robot-steering
```

## 一键安装脚本

创建安装脚本 `install_navigation_deps.sh`：

```bash
#!/bin/bash

echo "=========================================="
echo "CleanBot Navigation 依赖安装脚本"
echo "=========================================="

# 更新软件源
echo "更新软件源..."
sudo apt update

# 安装Nav2
echo "安装Nav2导航栈..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# 安装SLAM
echo "安装SLAM Toolbox..."
sudo apt install -y \
    ros-humble-slam-toolbox

# 安装传感器融合
echo "安装Robot Localization..."
sudo apt install -y \
    ros-humble-robot-localization

# 安装其他依赖
echo "安装其他依赖..."
sudo apt install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-diagnostic-updater \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rviz2

echo "=========================================="
echo "安装完成！"
echo "=========================================="

# 验证安装
echo "验证安装..."
ros2 pkg list | grep -E "navigation2|slam_toolbox|robot_localization"

echo ""
echo "如果上面显示了相关包，说明安装成功！"
echo "现在可以编译cleanbot_navigation包了："
echo "  cd ~/桌面/MOON/Electronic/CleanBot_ws"
echo "  colcon build --packages-select cleanbot_navigation"
echo "  source install/setup.bash"
```

### 使用安装脚本

```bash
# 保存脚本
cd ~/桌面/MOON/Electronic/CleanBot_ws/src/cleanbot_navigation
cat > install_navigation_deps.sh << 'EOF'
[上面的脚本内容]
EOF

# 添加执行权限
chmod +x install_navigation_deps.sh

# 运行安装
./install_navigation_deps.sh
```

## Python依赖（通常已安装）

```bash
# 如果缺少，手动安装
pip3 install numpy scipy
```

## 可选工具

### Cartographer（备选SLAM方案）

```bash
sudo apt install -y ros-humble-cartographer \
                    ros-humble-cartographer-ros
```

### Groot（行为树可视化）

```bash
sudo apt install -y ros-humble-groot
```

## 验证安装

### 检查已安装的包

```bash
# 检查Nav2
ros2 pkg list | grep nav2

# 检查SLAM Toolbox
ros2 pkg list | grep slam

# 检查Robot Localization
ros2 pkg list | grep robot_localization
```

### 检查关键可执行文件

```bash
# SLAM Toolbox
ros2 run slam_toolbox async_slam_toolbox_node --help

# AMCL
ros2 run nav2_amcl amcl --help

# Map Server
ros2 run nav2_map_server map_server --help

# Map Saver
ros2 run nav2_map_server map_saver_cli --help
```

## 编译cleanbot_navigation

```bash
cd ~/桌面/MOON/Electronic/CleanBot_ws

# 清理旧的编译文件（可选）
rm -rf build/cleanbot_navigation install/cleanbot_navigation

# 编译
colcon build --packages-select cleanbot_navigation

# 刷新环境
source install/setup.bash

# 验证
ros2 pkg list | grep cleanbot_navigation
ros2 run cleanbot_navigation navigation_mode_manager --help
```

## 环境配置

### 设置RMW（DDS实现）

```bash
# 在 ~/.bashrc 中添加
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

### 设置域ID（如果有多个ROS系统）

```bash
# 在 ~/.bashrc 中添加
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
source ~/.bashrc
```

## 故障排除

### 问题1：找不到包

```bash
# 重新索引ROS包
rosdep update

# 安装依赖
cd ~/桌面/MOON/Electronic/CleanBot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 问题2：编译错误

```bash
# 清理并重新编译
cd ~/桌面/MOON/Electronic/CleanBot_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### 问题3：权限问题

```bash
# 添加用户到dialout组（串口访问）
sudo usermod -aG dialout $USER

# 重新登录使生效
```

## 磁盘空间要求

- Nav2及相关包：~500MB
- SLAM Toolbox：~50MB
- 编译空间：~200MB
- 地图存储：根据使用情况，每个地图约1-10MB

**总计建议预留**：至少2GB空闲空间

## 系统要求

- **操作系统**：Ubuntu 22.04 LTS
- **ROS版本**：ROS 2 Humble Hawksbill
- **处理器**：Intel Core i3 或更高（ARM64也支持）
- **内存**：至少4GB RAM（推荐8GB）
- **存储**：至少10GB可用空间

## 网络要求

首次安装需要网络连接下载包，约需下载800MB-1GB数据。

如果网络较慢，可以考虑：
1. 使用国内镜像源
2. 提前下载.deb包
3. 使用离线安装包

## 完成检查清单

安装完成后，检查以下命令是否都能正常执行：

- [ ] `ros2 run slam_toolbox async_slam_toolbox_node --help`
- [ ] `ros2 run nav2_amcl amcl --help`
- [ ] `ros2 run nav2_map_server map_server --help`
- [ ] `ros2 pkg list | grep cleanbot_navigation`
- [ ] `rviz2`

如果全部通过，说明安装成功！可以开始使用导航系统了。

## 下一步

参考以下文档开始使用：

1. **快速开始**：`QUICK_START_NAVIGATION.md`
2. **详细功能**：`README.md`
3. **设计原理**：`docs/导航系统设计文档.md`

---

**有问题？** 查看故障排除章节或联系支持：3210676508@qq.com













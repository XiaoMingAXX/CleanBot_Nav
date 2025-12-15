# 地图存储目录

此目录用于存储SLAM建图生成的地图文件。

## 地图文件格式

每个地图由两个文件组成：

1. **`.yaml`文件**：地图元数据
   - 图像文件名
   - 分辨率
   - 原点位置
   - 占用阈值

2. **`.pgm`文件**：地图图像数据
   - PGM格式的灰度图像
   - 黑色=障碍物
   - 白色=自由空间
   - 灰色=未知区域

## 默认保存位置

虽然此目录可以存储地图，但系统默认将地图保存到：

```
~/cleanbot_maps/
```

这样可以避免地图文件被git追踪，并便于用户管理。

## 使用示例

### 保存地图

在Web界面的"建图模式"下点击"保存地图"按钮，地图将自动保存到：

```
~/cleanbot_maps/cleanbot_map_20251214_120000.yaml
~/cleanbot_maps/cleanbot_map_20251214_120000.pgm
```

### 加载地图

启动导航时指定地图文件：

```bash
ros2 launch cleanbot_navigation navigation_bringup.launch.py \
    map:=~/cleanbot_maps/cleanbot_map_20251214_120000.yaml
```

或在localization_only.launch.py中修改默认地图路径。

## 地图命名规范

建议使用以下命名方式：

- 带时间戳：`cleanbot_map_20251214_120000.yaml`（系统默认）
- 按区域：`living_room.yaml`, `kitchen.yaml`
- 按楼层：`floor_1.yaml`, `floor_2.yaml`
- 按版本：`office_v1.yaml`, `office_v2.yaml`

## 地图文件示例

### example_map.yaml

```yaml
image: example_map.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## 注意事项

1. **不要手动编辑`.pgm`文件**：使用SLAM系统生成
2. **保持两个文件同名**：`.yaml`和`.pgm`文件名必须匹配
3. **备份重要地图**：定期备份到云端或其他位置
4. **版本控制**：如果环境有重大变化，保存新版本地图

## 地图质量检查

### 使用RViz2查看地图

```bash
# 启动RViz2
rviz2

# 添加Map显示
# Topic: /map
# Fixed Frame: map
```

### 查看地图文件

```bash
# 查看地图元数据
cat ~/cleanbot_maps/your_map.yaml

# 查看地图图像（需要安装gimp或eog）
eog ~/cleanbot_maps/your_map.pgm
# 或
gimp ~/cleanbot_maps/your_map.pgm
```

## 地图优化

如果地图质量不佳，可以：

1. **重新建图**：慢速移动，多次经过同一区域
2. **手动编辑**：使用GIMP等工具修正小错误
3. **调整参数**：修改SLAM配置文件中的分辨率和阈值

## 地图存储空间

- 小房间（50m²）：约1-2MB
- 中型区域（200m²）：约5-8MB
- 大型区域（1000m²）：约20-30MB

定期清理不需要的旧地图文件。



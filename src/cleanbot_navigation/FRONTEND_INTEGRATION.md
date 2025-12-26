# 前端集成指南 - 清扫功能

本文档说明如何将清扫功能集成到前端控制界面。

## 前端需要实现的功能

### 1. 工作模式选择

提供四种工作模式的切换按钮或下拉框：

```javascript
const CLEANING_MODES = {
  STANDBY: 0,           // 待机模式（单点导航）
  EDGE_CLEANING: 1,      // 沿边模式
  BOUSTROPHEDON: 2,      // 弓形模式
  AUTO_WHOLE_HOUSE: 3    // 全屋模式
};

// 切换清扫模式
function setCleaningMode(mode) {
  const topic = new ROSLIB.Topic({
    ros: ros,
    name: '/cleaning/mode_cmd',
    messageType: 'std_msgs/UInt8'
  });
  
  const message = new ROSLIB.Message({
    data: mode
  });
  
  topic.publish(message);
  console.log('切换到清扫模式:', mode);
}
```

### 2. 清扫区域设置

#### 方式1: 地图上点击设置（推荐）

在地图上允许用户点击4个点定义清扫区域：

```javascript
let cleaningPoints = [];

// 监听地图点击事件
function onMapClick(x, y) {
  cleaningPoints.push([x, y]);
  
  // 在地图上显示标记
  drawPointMarker(x, y, cleaningPoints.length);
  
  if (cleaningPoints.length === 4) {
    // 已收集4个点，发送清扫区域
    sendCleaningArea(cleaningPoints);
    
    // 在地图上绘制清扫区域边框
    drawCleaningArea(cleaningPoints);
    
    // 重置
    cleaningPoints = [];
  }
}

// 发送清扫区域
function sendCleaningArea(points) {
  const topic = new ROSLIB.Topic({
    ros: ros,
    name: '/cleaning/area_cmd',
    messageType: 'std_msgs/String'
  });
  
  const areaData = {
    points: points
  };
  
  const message = new ROSLIB.Message({
    data: JSON.stringify(areaData)
  });
  
  topic.publish(message);
  console.log('清扫区域已设置:', points);
}
```

#### 方式2: 输入框设置

提供输入框让用户直接输入坐标：

```html
<div class="cleaning-area-input">
  <h3>设置清扫区域</h3>
  <div>
    <label>点1 (x, y):</label>
    <input type="number" id="x1" step="0.1"> 
    <input type="number" id="y1" step="0.1">
  </div>
  <div>
    <label>点2 (x, y):</label>
    <input type="number" id="x2" step="0.1"> 
    <input type="number" id="y2" step="0.1">
  </div>
  <div>
    <label>点3 (x, y):</label>
    <input type="number" id="x3" step="0.1"> 
    <input type="number" id="y3" step="0.1">
  </div>
  <div>
    <label>点4 (x, y):</label>
    <input type="number" id="x4" step="0.1"> 
    <input type="number" id="y4" step="0.1">
  </div>
  <button onclick="setCleaningAreaFromInputs()">设置清扫区域</button>
</div>
```

```javascript
function setCleaningAreaFromInputs() {
  const points = [
    [parseFloat(document.getElementById('x1').value), 
     parseFloat(document.getElementById('y1').value)],
    [parseFloat(document.getElementById('x2').value), 
     parseFloat(document.getElementById('y2').value)],
    [parseFloat(document.getElementById('x3').value), 
     parseFloat(document.getElementById('y3').value)],
    [parseFloat(document.getElementById('x4').value), 
     parseFloat(document.getElementById('y4').value)]
  ];
  
  sendCleaningArea(points);
}
```

#### 方式3: 预设区域

提供常用区域的快捷按钮：

```javascript
const PRESET_AREAS = {
  small: [[0.5, 0.5], [2.5, 0.5], [2.5, 2.5], [0.5, 2.5]],     // 2x2米
  medium: [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],     // 4x4米
  large: [[-1.0, -1.0], [5.0, -1.0], [5.0, 5.0], [-1.0, 5.0]]   // 6x6米
};

function setPresetArea(size) {
  sendCleaningArea(PRESET_AREAS[size]);
}
```

### 3. 状态监控

订阅清扫任务状态：

```javascript
// 订阅清扫模式状态
const modeStatusListener = new ROSLIB.Topic({
  ros: ros,
  name: '/cleaning/mode_status',
  messageType: 'std_msgs/UInt8'
});

modeStatusListener.subscribe((message) => {
  const mode = message.data;
  updateModeDisplay(mode);
  console.log('当前清扫模式:', mode);
});

// 订阅任务信息
const taskInfoListener = new ROSLIB.Topic({
  ros: ros,
  name: '/cleaning/task_info',
  messageType: 'std_msgs/String'
});

taskInfoListener.subscribe((message) => {
  const info = message.data;
  displayTaskInfo(info);
  
  // 解析状态
  if (info.includes('task_started')) {
    showNotification('清扫任务已开始');
  } else if (info.includes('task_completed')) {
    showNotification('清扫任务已完成');
  } else if (info.includes('task_failed')) {
    showNotification('清扫任务失败', 'error');
  }
});

// 订阅区域收集信息
const areaCollectorListener = new ROSLIB.Topic({
  ros: ros,
  name: '/cleaning/area_collector_info',
  messageType: 'std_msgs/String'
});

areaCollectorListener.subscribe((message) => {
  const info = message.data;
  
  if (info.startsWith('point_collected:')) {
    const count = info.split(':')[1];
    updatePointCollectionStatus(count);
  } else if (info === 'area_ready') {
    showNotification('清扫区域已设置');
  }
});
```

### 4. 清扫路径可视化

订阅并显示清扫路径：

```javascript
// 订阅弓形路径
const boustrophedonPathListener = new ROSLIB.Topic({
  ros: ros,
  name: '/cleaning/boustrophedon_path',
  messageType: 'nav_msgs/Path'
});

boustrophedonPathListener.subscribe((message) => {
  drawPath(message.poses, 'yellow', 'boustrophedon');
});

// 订阅沿边路径
const edgePathListener = new ROSLIB.Topic({
  ros: ros,
  name: '/cleaning/edge_path',
  messageType: 'nav_msgs/Path'
});

edgePathListener.subscribe((message) => {
  drawPath(message.poses, 'cyan', 'edge');
});

// 绘制路径
function drawPath(poses, color, type) {
  // 使用Canvas或SVG绘制路径
  const canvas = document.getElementById('map-canvas');
  const ctx = canvas.getContext('2d');
  
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  
  poses.forEach((pose, index) => {
    const x = worldToCanvas(pose.pose.position.x);
    const y = worldToCanvas(pose.pose.position.y);
    
    if (index === 0) {
      ctx.moveTo(x, y);
    } else {
      ctx.lineTo(x, y);
    }
  });
  
  ctx.stroke();
}
```

### 5. 覆盖地图显示

订阅并显示已清扫区域：

```javascript
// 订阅覆盖地图
const coverageMapListener = new ROSLIB.Topic({
  ros: ros,
  name: '/cleaning/coverage_map',
  messageType: 'nav_msgs/OccupancyGrid'
});

coverageMapListener.subscribe((message) => {
  drawCoverageMap(message);
});

// 绘制覆盖地图
function drawCoverageMap(mapData) {
  const canvas = document.getElementById('coverage-canvas');
  const ctx = canvas.getContext('2d');
  
  const width = mapData.info.width;
  const height = mapData.info.height;
  const resolution = mapData.info.resolution;
  
  // 绘制覆盖区域
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const index = y * width + x;
      const value = mapData.data[index];
      
      if (value === 100) {
        // 已覆盖区域（黑色半透明）
        ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
        const px = x * resolution;
        const py = y * resolution;
        ctx.fillRect(worldToCanvas(px), worldToCanvas(py), 
                     resolution * scale, resolution * scale);
      }
    }
  }
}
```

### 6. 清扫控制按钮

完整的清扫控制面板示例：

```html
<div class="cleaning-control-panel">
  <h2>清扫控制</h2>
  
  <!-- 模式选择 -->
  <div class="mode-selection">
    <h3>工作模式</h3>
    <button onclick="setCleaningMode(0)" class="mode-btn" id="mode-0">
      待机模式
    </button>
    <button onclick="setCleaningMode(1)" class="mode-btn" id="mode-1">
      沿边清扫
    </button>
    <button onclick="setCleaningMode(2)" class="mode-btn" id="mode-2">
      弓形清扫
    </button>
    <button onclick="setCleaningMode(3)" class="mode-btn" id="mode-3">
      全屋清扫
    </button>
  </div>
  
  <!-- 区域设置 -->
  <div class="area-setting">
    <h3>清扫区域</h3>
    <p>已选点数: <span id="point-count">0</span> / 4</p>
    <button onclick="startAreaSelection()">在地图上选择区域</button>
    <button onclick="clearAreaSelection()">清除选择</button>
    
    <!-- 预设区域 -->
    <div class="preset-areas">
      <h4>快速选择</h4>
      <button onclick="setPresetArea('small')">小区域 (2x2m)</button>
      <button onclick="setPresetArea('medium')">中区域 (4x4m)</button>
      <button onclick="setPresetArea('large')">大区域 (6x6m)</button>
    </div>
  </div>
  
  <!-- 状态显示 -->
  <div class="status-display">
    <h3>清扫状态</h3>
    <p>当前模式: <span id="current-mode">待机</span></p>
    <p>任务状态: <span id="task-status">空闲</span></p>
    <div id="coverage-progress">
      <label>覆盖进度:</label>
      <progress id="progress-bar" value="0" max="100"></progress>
      <span id="progress-text">0%</span>
    </div>
  </div>
  
  <!-- 路径显示控制 -->
  <div class="path-display">
    <h3>路径显示</h3>
    <label>
      <input type="checkbox" id="show-boustrophedon" checked>
      显示弓形路径
    </label>
    <label>
      <input type="checkbox" id="show-edge" checked>
      显示沿边路径
    </label>
    <label>
      <input type="checkbox" id="show-coverage" checked>
      显示覆盖地图
    </label>
  </div>
  
  <!-- 清扫控制 -->
  <div class="cleaning-actions">
    <h3>操作</h3>
    <button onclick="startCleaning()" class="btn-success">开始清扫</button>
    <button onclick="pauseCleaning()" class="btn-warning">暂停</button>
    <button onclick="stopCleaning()" class="btn-danger">停止</button>
    <button onclick="clearCoverage()" class="btn-info">清除覆盖记录</button>
  </div>
</div>
```

```css
.cleaning-control-panel {
  padding: 20px;
  background: #f5f5f5;
  border-radius: 8px;
  max-width: 400px;
}

.mode-btn {
  display: block;
  width: 100%;
  padding: 10px;
  margin: 5px 0;
  border: 2px solid #ccc;
  background: white;
  cursor: pointer;
  border-radius: 4px;
  transition: all 0.3s;
}

.mode-btn.active {
  background: #4CAF50;
  color: white;
  border-color: #4CAF50;
}

.mode-btn:hover {
  background: #e0e0e0;
}

.btn-success { background: #4CAF50; color: white; }
.btn-warning { background: #FF9800; color: white; }
.btn-danger { background: #f44336; color: white; }
.btn-info { background: #2196F3; color: white; }
```

### 7. 清除覆盖地图服务

```javascript
function clearCoverage() {
  const clearService = new ROSLIB.Service({
    ros: ros,
    name: '/cleaning/clear_coverage',
    serviceType: 'std_srvs/Trigger'
  });
  
  const request = new ROSLIB.ServiceRequest({});
  
  clearService.callService(request, (result) => {
    if (result.success) {
      showNotification('覆盖地图已清除');
    } else {
      showNotification('清除失败: ' + result.message, 'error');
    }
  });
}
```

## 完整的集成示例

```javascript
class CleaningController {
  constructor(ros) {
    this.ros = ros;
    this.currentMode = 0;
    this.cleaningPoints = [];
    this.setupTopics();
    this.setupServices();
  }
  
  setupTopics() {
    // 发布话题
    this.modeCmdTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/cleaning/mode_cmd',
      messageType: 'std_msgs/UInt8'
    });
    
    this.areaCmdTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/cleaning/area_cmd',
      messageType: 'std_msgs/String'
    });
    
    // 订阅话题
    this.modeStatusTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/cleaning/mode_status',
      messageType: 'std_msgs/UInt8'
    });
    
    this.modeStatusTopic.subscribe((msg) => {
      this.onModeStatusUpdate(msg.data);
    });
    
    // ... 订阅其他话题
  }
  
  setupServices() {
    this.clearCoverageService = new ROSLIB.Service({
      ros: this.ros,
      name: '/cleaning/clear_coverage',
      serviceType: 'std_srvs/Trigger'
    });
  }
  
  setMode(mode) {
    const msg = new ROSLIB.Message({ data: mode });
    this.modeCmdTopic.publish(msg);
  }
  
  setCleaningArea(points) {
    const areaData = { points: points };
    const msg = new ROSLIB.Message({ 
      data: JSON.stringify(areaData) 
    });
    this.areaCmdTopic.publish(msg);
  }
  
  addCleaningPoint(x, y) {
    this.cleaningPoints.push([x, y]);
    
    if (this.cleaningPoints.length === 4) {
      this.setCleaningArea(this.cleaningPoints);
      this.cleaningPoints = [];
      return true;
    }
    
    return false;
  }
  
  clearCoverage() {
    const request = new ROSLIB.ServiceRequest({});
    this.clearCoverageService.callService(request, (result) => {
      console.log('Clear coverage:', result);
    });
  }
  
  onModeStatusUpdate(mode) {
    this.currentMode = mode;
    // 更新UI
    this.updateModeDisplay();
  }
  
  updateModeDisplay() {
    const modeNames = ['待机', '沿边', '弓形', '全屋'];
    document.getElementById('current-mode').textContent = 
      modeNames[this.currentMode];
    
    // 高亮当前模式按钮
    document.querySelectorAll('.mode-btn').forEach((btn, index) => {
      btn.classList.toggle('active', index === this.currentMode);
    });
  }
}

// 使用
const cleaningController = new CleaningController(ros);

// 在地图点击时
canvas.addEventListener('click', (event) => {
  const {x, y} = getWorldCoords(event);
  const isComplete = cleaningController.addCleaningPoint(x, y);
  
  if (isComplete) {
    alert('清扫区域已设置！');
  }
});
```

## ROS Bridge配置

确保在rosbridge中包含清扫相关的话题：

```yaml
# rosbridge配置
topics:
  - /cleaning/mode_cmd
  - /cleaning/mode_status
  - /cleaning/area_cmd
  - /cleaning/area_polygon
  - /cleaning/boustrophedon_path
  - /cleaning/edge_path
  - /cleaning/coverage_map
  - /cleaning/task_info

services:
  - /cleaning/compute_boustrophedon_path
  - /cleaning/compute_edge_path
  - /cleaning/clear_coverage
```

## 注意事项

1. **坐标系转换**: 前端地图坐标需要转换为ROS世界坐标
2. **状态同步**: 定期订阅状态话题保持前端与后端同步
3. **错误处理**: 处理清扫失败、路径规划失败等异常情况
4. **用户体验**: 提供清晰的状态反馈和操作提示

## 测试建议

1. 先在RViz中测试功能是否正常
2. 使用rosbridge_suite测试与前端的通信
3. 逐步集成各个功能模块
4. 进行完整的端到端测试


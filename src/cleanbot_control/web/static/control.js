// WebSocket连接
let ws = null;
let reconnectInterval = null;

// 摇杆控制变量
let isDragging = false;
let joystickCenter = { x: 0, y: 0 };
let currentLinear = 0;
let currentAngular = 0;

// 地图相关变量
let mapCanvas = null;
let mapCtx = null;
let robotPose = { x: 0, y: 0, theta: 0 };
let goalPose = null;
let path = [];
let mapData = null;
let mapScale = 20; // 像素/米
let mapOffset = { x: 0, y: 0 };

// 工作模式映射
const workModeNames = ['待机', '自动全屋', '沿边', '弓形', '遥控', '回充'];
const dockStatusNames = ['无', '接近', '成功', '失败'];
let currentMode = 0;

// 导航模式映射
const navigationModeNames = ['手动', '建图', '导航'];
let currentNavigationMode = 0;  // 0=手动, 1=建图, 2=导航
let joystickEnabled = true;
let availableMaps = [];  // 可用地图列表
let modeSwitchTimeoutId = null;  // 模式切换超时定时器

// 初始化
window.onload = function() {
    initMap();
    connectWebSocket();
    initJoystick();
    // 注意：不在这里刷新地图列表，等WebSocket连接成功后再刷新
    addLog('前端界面加载完成');
};

// ==================== 地图初始化和绘制 ====================

function initMap() {
    mapCanvas = document.getElementById('map-canvas');
    if (!mapCanvas) {
        console.error('Canvas element not found');
        return;
    }
    
    mapCtx = mapCanvas.getContext('2d');
    
    // 设置canvas尺寸
    mapCanvas.width = mapCanvas.clientWidth;
    mapCanvas.height = mapCanvas.clientHeight;
    
    // 设置地图中心偏移
    mapOffset.x = mapCanvas.width / 2;
    mapOffset.y = mapCanvas.height / 2;
    
    // 绑定点击事件用于设置目标点
    mapCanvas.addEventListener('click', handleMapClick);
    
    // 定时刷新地图
    setInterval(drawMap, 100); // 10Hz
    
    addLog('地图系统初始化完成');
}

function handleMapClick(event) {
    const rect = mapCanvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    
    // 转换为世界坐标
    const worldX = (x - mapOffset.x) / mapScale;
    const worldY = -(y - mapOffset.y) / mapScale; // Y轴反向
    
    // 设置目标点
    setGoal(worldX, worldY);
}

function setGoal(x, y) {
    goalPose = { x: x, y: y };
    addLog(`设置目标点: (${x.toFixed(2)}, ${y.toFixed(2)})`);
    
    // 发送目标点到后端
    sendWebSocketMessage({
        type: 'set_goal',
        x: x,
        y: y
    });
}

function drawMap() {
    if (!mapCtx) return;
    
    // 清空画布
    mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    
    // 绘制坐标网格
    drawGrid();
    
    // 绘制地图数据（如果有）
    if (mapData) {
        drawMapData();
    }
    
    // 绘制路径
    if (path && path.length > 0) {
        drawPath();
    }
    
    // 绘制目标点
    if (goalPose) {
        drawGoal();
    }
    
    // 绘制机器人
    drawRobot();
}

function drawGrid() {
    mapCtx.strokeStyle = 'rgba(0, 217, 255, 0.1)';
    mapCtx.lineWidth = 0.5;
    
    // 绘制垂直线
    for (let x = 0; x < mapCanvas.width; x += 50) {
        mapCtx.beginPath();
        mapCtx.moveTo(x, 0);
        mapCtx.lineTo(x, mapCanvas.height);
        mapCtx.stroke();
    }
    
    // 绘制水平线
    for (let y = 0; y < mapCanvas.height; y += 50) {
        mapCtx.beginPath();
        mapCtx.moveTo(0, y);
        mapCtx.lineTo(mapCanvas.width, y);
        mapCtx.stroke();
    }
    
    // 绘制中心轴
    mapCtx.strokeStyle = 'rgba(0, 217, 255, 0.3)';
    mapCtx.lineWidth = 2;
    
    // X轴
    mapCtx.beginPath();
    mapCtx.moveTo(0, mapOffset.y);
    mapCtx.lineTo(mapCanvas.width, mapOffset.y);
    mapCtx.stroke();
    
    // Y轴
    mapCtx.beginPath();
    mapCtx.moveTo(mapOffset.x, 0);
    mapCtx.lineTo(mapOffset.x, mapCanvas.height);
    mapCtx.stroke();
}

function drawMapData() {
    // 预留给后续的地图数据绘制
    // mapData应该包含栅格地图信息
    // 这里可以绘制已知的障碍物等
}

function drawPath() {
    if (path.length < 2) return;
    
    mapCtx.strokeStyle = 'rgba(255, 170, 0, 0.8)';
    mapCtx.lineWidth = 3;
    mapCtx.setLineDash([10, 5]);
    
    mapCtx.beginPath();
    const start = worldToScreen(path[0].x, path[0].y);
    mapCtx.moveTo(start.x, start.y);
    
    for (let i = 1; i < path.length; i++) {
        const point = worldToScreen(path[i].x, path[i].y);
        mapCtx.lineTo(point.x, point.y);
    }
    
    mapCtx.stroke();
    mapCtx.setLineDash([]);
}

function drawGoal() {
    const screen = worldToScreen(goalPose.x, goalPose.y);
    
    // 绘制目标点
    mapCtx.fillStyle = 'rgba(0, 255, 136, 0.6)';
    mapCtx.beginPath();
    mapCtx.arc(screen.x, screen.y, 15, 0, Math.PI * 2);
    mapCtx.fill();
    
    mapCtx.strokeStyle = 'rgba(0, 255, 136, 1)';
    mapCtx.lineWidth = 3;
    mapCtx.beginPath();
    mapCtx.arc(screen.x, screen.y, 15, 0, Math.PI * 2);
    mapCtx.stroke();
    
    // 绘制十字标记
    mapCtx.beginPath();
    mapCtx.moveTo(screen.x - 10, screen.y);
    mapCtx.lineTo(screen.x + 10, screen.y);
    mapCtx.moveTo(screen.x, screen.y - 10);
    mapCtx.lineTo(screen.x, screen.y + 10);
    mapCtx.stroke();
}

function drawRobot() {
    const screen = worldToScreen(robotPose.x, robotPose.y);
    
    mapCtx.save();
    mapCtx.translate(screen.x, screen.y);
    mapCtx.rotate(-robotPose.theta); // 注意负号，因为canvas的Y轴是向下的
    
    // 绘制机器人本体（圆形）
    mapCtx.fillStyle = 'rgba(0, 217, 255, 0.6)';
    mapCtx.beginPath();
    mapCtx.arc(0, 0, 20, 0, Math.PI * 2);
    mapCtx.fill();
    
    mapCtx.strokeStyle = 'rgba(0, 217, 255, 1)';
    mapCtx.lineWidth = 3;
    mapCtx.beginPath();
    mapCtx.arc(0, 0, 20, 0, Math.PI * 2);
    mapCtx.stroke();
    
    // 绘制方向指示器
    mapCtx.fillStyle = 'rgba(255, 255, 255, 0.9)';
    mapCtx.beginPath();
    mapCtx.moveTo(20, 0);
    mapCtx.lineTo(10, -8);
    mapCtx.lineTo(10, 8);
    mapCtx.closePath();
    mapCtx.fill();
    
    mapCtx.restore();
    
    // 绘制轨迹光晕效果
    mapCtx.shadowBlur = 20;
    mapCtx.shadowColor = 'rgba(0, 217, 255, 0.5)';
    mapCtx.fillStyle = 'rgba(0, 217, 255, 0.1)';
    mapCtx.beginPath();
    mapCtx.arc(screen.x, screen.y, 30, 0, Math.PI * 2);
    mapCtx.fill();
    mapCtx.shadowBlur = 0;
}

function worldToScreen(x, y) {
    return {
        x: x * mapScale + mapOffset.x,
        y: -y * mapScale + mapOffset.y
    };
}

function screenToWorld(x, y) {
    return {
        x: (x - mapOffset.x) / mapScale,
        y: -(y - mapOffset.y) / mapScale
    };
}

// ==================== WebSocket连接 ====================

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    // 修复：确保使用正确的端口，如果location.port为空，使用默认的8080
    const port = window.location.port || '8080';
    const hostname = window.location.hostname || 'localhost';
    const wsUrl = `${protocol}//${hostname}:${port}/ws`;
    
    addLog('尝试连接WebSocket: ' + wsUrl);
    
    try {
        ws = new WebSocket(wsUrl);
        
        ws.onopen = function() {
            updateConnectionStatus(true);
            addLog('WebSocket连接成功');
            if (reconnectInterval) {
                clearInterval(reconnectInterval);
                reconnectInterval = null;
            }
            // WebSocket连接成功后，刷新地图列表
            setTimeout(() => {
                refreshMapList();
            }, 500);
        };
        
        ws.onmessage = function(event) {
            handleWebSocketMessage(event.data);
            // 更新最后接收时间指示器
            if (!window.messageCount) window.messageCount = 0;
            window.messageCount++;
            if (window.messageCount <= 5) {
                addLog(`收到第${window.messageCount}条WebSocket消息`);
            }
        };
        
        ws.onerror = function(error) {
            addLog('WebSocket错误: ' + error);
            console.error('WebSocket错误详情:', error);
        };
        
        ws.onclose = function(event) {
            updateConnectionStatus(false);
            addLog('WebSocket断开连接 (code: ' + event.code + ')');
            
            // 自动重连
            if (!reconnectInterval) {
                reconnectInterval = setInterval(function() {
                    addLog('尝试重新连接...');
                    connectWebSocket();
                }, 3000);
            }
        };
    } catch (error) {
        addLog('无法创建WebSocket连接: ' + error);
        console.error('WebSocket创建失败:', error);
    }
}

function updateConnectionStatus(connected) {
    const wsStatus = document.getElementById('ws-status');
    const wsText = document.getElementById('ws-text');
    
    if (connected) {
        wsStatus.className = 'status-indicator status-connected';
        wsText.textContent = '已连接';
    } else {
        wsStatus.className = 'status-indicator status-disconnected';
        wsText.textContent = '断开';
    }
}

function handleWebSocketMessage(data) {
    try {
        const message = JSON.parse(data);
        
        if (message.type === 'state_update') {
            const state = message.data;
            
            updateRobotState(message.data);
        } else if (message.type === 'info') {
            addLog('✅ ' + message.message);
        } else if (message.type === 'map_update') {
            updateMapData(message.data);
        } else if (message.type === 'path_update') {
            updatePath(message.data);
        } else if (message.type === 'navigation_info') {
            handleNavigationInfo(message.message);
        } else if (message.type === 'map_list') {
            updateMapList(message.maps);
        } else if (message.type === 'error') {
            addLog('❌ 错误: ' + message.message);
            console.error('[WebSocket] 错误:', message.message);
        } else {
            console.warn('[WebSocket] 未知消息类型:', message.type);
        }
    } catch (error) {
        console.error('❌ 解析消息失败:', error);
        console.error('原始数据:', data);
        addLog('消息解析错误: ' + error.message);
    }
}


function sendWebSocketMessage(message) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        try {
            ws.send(JSON.stringify(message));
        } catch (error) {
            console.error('发送WebSocket消息失败:', error);
            // 不要每次都打印到日志，避免刷屏
            if (message.type !== 'cmd_vel') {
                addLog('WebSocket发送失败: ' + error.message);
            }
        }
    } else {
        // 只对非速度命令打印警告
        if (message.type !== 'cmd_vel') {
            addLog('WebSocket未连接，无法发送消息');
        }
    }
}

// ==================== 机器人状态更新 ====================

function updateRobotState(state) {
    // USB连接状态
    const usbStatus = document.getElementById('usb-status');
    const usbText = document.getElementById('usb-text');
    if (state.usb_connected) {
        usbStatus.className = 'status-indicator status-connected';
        usbText.textContent = '已连接';
    } else {
        usbStatus.className = 'status-indicator status-disconnected';
        usbText.textContent = '断开';
    }
    
    // 通讯质量
    document.getElementById('comm-quality').textContent = state.comm_quality || '-';
    
    // IMU数据
    if (state.imu) {
        document.getElementById('imu-ax').textContent = state.imu.linear_acceleration.x.toFixed(2) + ' m/s²';
        document.getElementById('imu-ay').textContent = state.imu.linear_acceleration.y.toFixed(2) + ' m/s²';
        document.getElementById('imu-az').textContent = state.imu.linear_acceleration.z.toFixed(2) + ' m/s²';
        document.getElementById('imu-gz').textContent = state.imu.angular_velocity.z.toFixed(2) + ' rad/s';
    }
    
    // 轮速数据
    if (state.wheels) {
        document.getElementById('wheel-left-pos').textContent = state.wheels.left_position.toFixed(2) + ' rad';
        document.getElementById('wheel-left-vel').textContent = state.wheels.left_velocity.toFixed(2) + ' rad/s';
        document.getElementById('wheel-right-pos').textContent = state.wheels.right_position.toFixed(2) + ' rad';
        document.getElementById('wheel-right-vel').textContent = state.wheels.right_velocity.toFixed(2) + ' rad/s';
    }
    
    // 传感器状态
    if (state.sensors) {
        updateSensorStatus('sensor-bumper-left', '左碰撞', state.sensors.bumper_left);
        updateSensorStatus('sensor-bumper-right', '右碰撞', state.sensors.bumper_right);
        updateSensorStatus('sensor-ir0', '红外0', state.sensors.ir_down0);
        updateSensorStatus('sensor-ir1', '红外1', state.sensors.ir_down1);
        updateSensorStatus('sensor-ir2', '红外2', state.sensors.ir_down2);
        
        const dockStatus = dockStatusNames[state.sensors.dock_status] || '未知';
        document.getElementById('dock-status').textContent = dockStatus;
    }
    
    // 里程计数据
    if (state.odometry) {
        document.getElementById('odom-x').textContent = state.odometry.position.x.toFixed(2) + ' m';
        document.getElementById('odom-y').textContent = state.odometry.position.y.toFixed(2) + ' m';
        document.getElementById('odom-vx').textContent = state.odometry.linear_velocity.x.toFixed(2) + ' m/s';
        document.getElementById('odom-wz').textContent = state.odometry.angular_velocity.z.toFixed(2) + ' rad/s';
        
        // 更新机器人位置用于地图显示
        robotPose.x = state.odometry.position.x;
        robotPose.y = state.odometry.position.y;
        
        // 从四元数转换为欧拉角
        const q = state.odometry.orientation;
        robotPose.theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        
        // 更新地图叠加层显示
        document.getElementById('robot-x').textContent = robotPose.x.toFixed(2);
        document.getElementById('robot-y').textContent = robotPose.y.toFixed(2);
        document.getElementById('robot-theta').textContent = (robotPose.theta * 180 / Math.PI).toFixed(1);
    }
    
    // 系统状态
    if (state.system_status && state.system_status.includes('available_ports:')) {
        const ports = state.system_status.split(':')[1].split(',');
        updatePortList(ports);
    } else if (state.system_status && state.system_status.includes('connected:')) {
        const port = state.system_status.split(':')[1];
        addLog('已连接到串口: ' + port);
    }
}

function updateMapData(data) {
    // 预留接口：接收地图数据
    mapData = data;
    addLog('接收到地图数据');
}

function updatePath(data) {
    // 预留接口：接收路径规划数据
    path = data.path || [];
    addLog(`接收到路径，共${path.length}个点`);
}

function updateSensorStatus(elementId, label, status) {
    const element = document.getElementById(elementId);
    if (status === 0) {
        element.className = 'sensor-item';
        element.textContent = label + ': 正常';
    } else {
        element.className = 'sensor-item sensor-active';
        element.textContent = label + ': 触发';
    }
}

function updatePortList(ports) {
    const select = document.getElementById('port-select');
    select.innerHTML = '<option value="">选择串口...</option>';
    
    ports.forEach(port => {
        if (port.trim()) {
            const option = document.createElement('option');
            option.value = port.trim();
            option.textContent = port.trim();
            select.appendChild(option);
        }
    });
    
    addLog('扫描到 ' + ports.length + ' 个串口');
}

// ==================== 摇杆控制 ====================

function initJoystick() {
    const joystick = document.getElementById('joystick');
    const handle = document.getElementById('joystick-handle');
    
    const rect = joystick.getBoundingClientRect();
    joystickCenter.x = rect.width / 2;
    joystickCenter.y = rect.height / 2;
    
    // 鼠标事件
    handle.addEventListener('mousedown', startDrag);
    document.addEventListener('mousemove', drag);
    document.addEventListener('mouseup', endDrag);
    
    // 触摸事件
    handle.addEventListener('touchstart', startDrag);
    document.addEventListener('touchmove', drag);
    document.addEventListener('touchend', endDrag);
    
    // 定时发送速度命令
    setInterval(sendVelocityCommand, 100);  // 10Hz
}

function startDrag(e) {
    e.preventDefault();
    isDragging = true;
    console.log('[摇杆调试] 开始拖拽, 摇杆启用:', joystickEnabled, '导航模式:', currentNavigationMode);
}

// 添加拖拽计数器用于调试
let dragEventCount = 0;
let lastDragEventTime = 0;

function drag(e) {
    if (!isDragging) {
        // 调试：拖拽事件在非拖拽状态下触发
        console.log('[摇杆调试] drag事件触发但isDragging=false');
        return;
    }
    
    e.preventDefault();
    const joystick = document.getElementById('joystick');
    const handle = document.getElementById('joystick-handle');
    const rect = joystick.getBoundingClientRect();
    
    let clientX, clientY;
    if (e.type.includes('touch')) {
        clientX = e.touches[0].clientX;
        clientY = e.touches[0].clientY;
    } else {
        clientX = e.clientX;
        clientY = e.clientY;
    }
    
    let x = clientX - rect.left - joystickCenter.x;
    let y = clientY - rect.top - joystickCenter.y;
    
    // 限制在圆形区域内
    const maxRadius = joystickCenter.x - 35;
    const distance = Math.sqrt(x * x + y * y);
    
    if (distance > maxRadius) {
        const angle = Math.atan2(y, x);
        x = maxRadius * Math.cos(angle);
        y = maxRadius * Math.sin(angle);
    }
    
    // 更新摇杆位置
    handle.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
    
    // 计算速度 (-y是前进方向，x是转向)
    let linear = -y / maxRadius * 0.8;  // 最大线速度0.8m/s
    let angular = -x / maxRadius * 2.0;  // 最大角速度2.0rad/s
    
    // 应用死区（deadzone）减少抖动
    const linearDeadzone = 0.05;  // 5%的死区
    const angularDeadzone = 0.1;  // 10%的死区
    
    if (Math.abs(linear) < linearDeadzone) {
        linear = 0;
    }
    if (Math.abs(angular) < angularDeadzone) {
        angular = 0;
    }
    
    currentLinear = linear;
    currentAngular = angular;
    
    // 调试：记录drag事件频率
    dragEventCount++;
    const now = Date.now();
    if (now - lastDragEventTime > 5000) {
        console.log(`[摇杆调试] drag事件触发了${dragEventCount}次，平均${(dragEventCount / 5).toFixed(1)}次/秒`);
        dragEventCount = 0;
        lastDragEventTime = now;
    }
    
    // 更新显示
    document.getElementById('linear-vel').textContent = currentLinear.toFixed(2) + ' m/s';
    document.getElementById('angular-vel').textContent = currentAngular.toFixed(2) + ' rad/s';
}

function endDrag(e) {
    if (!isDragging) return;
    
    isDragging = false;
    
    // 调试：记录endDrag触发原因
    const eventType = e ? e.type : 'unknown';
    console.log(`[摇杆调试] 结束拖拽 (事件:${eventType}), 发送停止命令`);
    
    // 回到中心
    const handle = document.getElementById('joystick-handle');
    handle.style.transform = 'translate(-50%, -50%)';
    
    currentLinear = 0;
    currentAngular = 0;
    
    // 更新显示
    document.getElementById('linear-vel').textContent = '0.00 m/s';
    document.getElementById('angular-vel').textContent = '0.00 rad/s';
    
    // 立即发送停止命令
    sendVelocityCommand();
}

// 上一次发送的速度值（用于检测变化）
let lastSentLinear = 0;
let lastSentAngular = 0;
let cmdVelSendCount = 0;  // 发送计数器
let lastCmdVelLogTime = 0;  // 上次打印日志的时间
let zeroVelSendCount = 0;  // 0速度发送计数
let nonZeroVelSendCount = 0;  // 非0速度发送计数

function sendVelocityCommand() {
    // 在导航模式下禁用摇杆控制
    if (!joystickEnabled) {
        // 调试：摇杆被禁用
        if (currentLinear !== 0 || currentAngular !== 0) {
            console.log('[摇杆调试] 摇杆被禁用，当前模式:', currentNavigationMode, '速度:', currentLinear, currentAngular);
        }
        return;
    }
    
    // 速度改变时必须发送
    const velocityChanged = (currentLinear !== lastSentLinear || currentAngular !== lastSentAngular);
    
    // 速度不为0时也要持续发送（保持控制器活跃，避免超时停止）
    const isMoving = (currentLinear !== 0 || currentAngular !== 0);
    
    // 只在以下情况发送：
    // 1. 速度改变了（包括从0变到非0，或从非0变到0）
    // 2. 正在移动（速度不为0）需要持续发送保持控制
    if (velocityChanged || isMoving) {
        const msg = {
            type: 'cmd_vel',
            linear: currentLinear,
            angular: currentAngular
        };
        
        // 调试：记录发送状态
        cmdVelSendCount++;
        if (currentLinear === 0 && currentAngular === 0) {
            zeroVelSendCount++;
            // 调试：发送0速度时记录isDragging状态
            console.log(`[摇杆调试] ⚠️ 发送0速度! isDragging=${isDragging}, 已发送0速度${zeroVelSendCount}次`);
        } else {
            nonZeroVelSendCount++;
        }
        
        const now = Date.now();
        
        // 每5秒打印一次统计信息（避免刷屏）
        if (now - lastCmdVelLogTime > 5000) {
            if (isMoving || zeroVelSendCount > 0) {
                console.log(`[摇杆调试] 统计: 总${cmdVelSendCount}条, 非0:${nonZeroVelSendCount}条, 0:${zeroVelSendCount}条, WebSocket:${ws ? ws.readyState : 'null'}, 当前速度: (${currentLinear.toFixed(2)}, ${currentAngular.toFixed(2)}), 拖拽中:${isDragging}`);
                // 重置0速度计数
                zeroVelSendCount = 0;
                nonZeroVelSendCount = 0;
            }
            lastCmdVelLogTime = now;
        }
        
        // 速度改变时立即打印
        if (velocityChanged) {
            console.log(`[摇杆调试] 速度改变: (${lastSentLinear.toFixed(2)}, ${lastSentAngular.toFixed(2)}) -> (${currentLinear.toFixed(2)}, ${currentAngular.toFixed(2)}) [isDragging=${isDragging}]`);
        }
        
        sendWebSocketMessage(msg);
        
        // 记录已发送的速度
        lastSentLinear = currentLinear;
        lastSentAngular = currentAngular;
    }
}

// ==================== 控制命令 ====================

function stopRobot() {
    currentLinear = 0;
    currentAngular = 0;
    sendVelocityCommand();
    addLog('紧急停止');
}

function setWorkMode(mode) {
    // 更新按钮状态
    for (let i = 0; i < 6; i++) {
        const btn = document.getElementById(`mode-${i}`);
        if (btn) {
            if (i === mode) {
                btn.classList.add('active');
            } else {
                btn.classList.remove('active');
            }
        }
    }
    
    currentMode = mode;
    
    sendWebSocketMessage({
        type: 'control_cmd',
        data: {
            work_mode: mode,
            side_brush_left: parseInt(document.getElementById('brush-left').value),
            side_brush_right: parseInt(document.getElementById('brush-right').value),
            fan_level: parseInt(document.getElementById('fan').value),
            water_level: parseInt(document.getElementById('water').value),
            need_ack: 1
        }
    });
    
    document.getElementById('work-mode').textContent = workModeNames[mode] || '未知';
    addLog('切换工作模式: ' + workModeNames[mode]);
}

function updateActuator() {
    const brushLeft = parseInt(document.getElementById('brush-left').value);
    const brushRight = parseInt(document.getElementById('brush-right').value);
    const fan = parseInt(document.getElementById('fan').value);
    const water = parseInt(document.getElementById('water').value);
    
    document.getElementById('brush-left-val').textContent = brushLeft;
    document.getElementById('brush-right-val').textContent = brushRight;
    document.getElementById('fan-val').textContent = fan;
    document.getElementById('water-val').textContent = water;
    
    sendWebSocketMessage({
        type: 'control_cmd',
        data: {
            work_mode: currentMode,
            side_brush_left: brushLeft,
            side_brush_right: brushRight,
            fan_level: fan,
            water_level: water,
            need_ack: 0
        }
    });
}

function scanPorts() {
    sendWebSocketMessage({
        type: 'scan_ports'
    });
    addLog('扫描串口...');
}

function connectPort() {
    const port = document.getElementById('port-select').value;
    if (!port) {
        addLog('请先选择串口');
        return;
    }
    
    sendWebSocketMessage({
        type: 'connect_port',
        port: port
    });
    addLog('连接串口: ' + port);
}

// ==================== 日志系统 ====================

function addLog(message) {
    const logContainer = document.getElementById('log-container');
    const entry = document.createElement('div');
    entry.className = 'log-entry';
    
    const time = new Date().toLocaleTimeString();
    entry.textContent = `[${time}] ${message}`;
    
    logContainer.appendChild(entry);
    
    // 限制日志条数
    while (logContainer.children.length > 50) {
        logContainer.removeChild(logContainer.firstChild);
    }
    
    // 滚动到底部
    logContainer.scrollTop = logContainer.scrollHeight;
}

// ==================== 导航模式控制 ====================

// 设置导航模式
function setNavigationMode(mode) {
    // 更新按钮状态
    for (let i = 0; i < 3; i++) {
        const btn = document.getElementById(`nav-mode-${i}`);
        if (btn) {
            if (i === mode) {
                btn.classList.add('active');
            } else {
                btn.classList.remove('active');
            }
        }
    }
    
    // 更新当前导航模式变量！！！
    currentNavigationMode = mode;
    
    // 显示启动中状态（等待真实反馈）
    const stateElement = document.getElementById('nav-mode-state');
    stateElement.textContent = '⏳ 切换中...';
    stateElement.style.color = 'var(--warning)';
    
    // 清除之前的超时定时器
    if (modeSwitchTimeoutId) {
        clearTimeout(modeSwitchTimeoutId);
    }
    
    // 设置超时保护（20秒后如果还没有反馈，自动恢复）
    modeSwitchTimeoutId = setTimeout(() => {
        if (stateElement.textContent === '⏳ 切换中...') {
            stateElement.textContent = '❌ 切换超时';
            stateElement.style.color = 'var(--danger)';
            addLog('❌ 模式切换超时（20秒未响应），可能是节点启动时间过长');
            console.error('[导航调试] 模式切换超时！');
            
            // 3秒后恢复就绪状态
            setTimeout(() => {
                stateElement.textContent = '✅ 就绪';
                stateElement.style.color = 'var(--success)';
            }, 3000);
        }
    }, 20000);  // 20秒超时
    
    // 发送模式切换命令到ROS
    sendWebSocketMessage({
        type: 'navigation_mode',
        mode: mode
    });
    
    // 更新UI显示
    document.getElementById('nav-mode-status').textContent = navigationModeNames[mode];
    
    // 修复bug：只有导航模式下禁用摇杆，建图模式需要摇杆！
    joystickEnabled = (mode !== 2);  // 只有导航模式(2)禁用摇杆
    
    // 更新信息提示
    const infoElement = document.getElementById('nav-info');
    if (mode === 0) {  // 手动模式
        infoElement.textContent = '✅ 摇杆控制已启用';
    } else if (mode === 1) {  // 建图模式
        infoElement.textContent = '✅ 摇杆可用，驾驶建图';
    } else {  // 导航模式
        infoElement.textContent = '⚠️ 自主导航中，摇杆已禁用';
    }
    
    addLog(`📡 请求切换导航模式: ${navigationModeNames[mode]} (摇杆: ${joystickEnabled ? '启用' : '禁用'})`);
}

// 处理导航信息（模式切换反馈、地图保存反馈等）
function handleNavigationInfo(info) {
    const stateElement = document.getElementById('nav-mode-state');
    
    // 调试：打印收到的所有导航信息
    console.log(`[导航调试] 收到导航信息: ${info}`);
    
    if (info.startsWith('mode_changed:')) {
        // 模式切换完成
        const modeName = info.split(':')[1];
        
        // 清除超时定时器
        if (modeSwitchTimeoutId) {
            clearTimeout(modeSwitchTimeoutId);
            modeSwitchTimeoutId = null;
        }
        
        stateElement.textContent = '✅ 就绪';
        stateElement.style.color = 'var(--success)';
        addLog(`✅ 模式切换完成: ${modeName}`);
    } else if (info.startsWith('mode_failed:')) {
        // 模式切换失败
        const error = info.split(':')[1] || '未知错误';
        
        // 清除超时定时器
        if (modeSwitchTimeoutId) {
            clearTimeout(modeSwitchTimeoutId);
            modeSwitchTimeoutId = null;
        }
        
        stateElement.textContent = '❌ 切换失败';
        stateElement.style.color = 'var(--danger)';
        addLog(`❌ 模式切换失败: ${error}`);
        console.error(`[导航调试] 模式切换失败: ${error}`);
        
        // 3秒后恢复就绪状态
        setTimeout(() => {
            stateElement.textContent = '✅ 就绪';
            stateElement.style.color = 'var(--success)';
        }, 3000);
    } else if (info.startsWith('map_saved:')) {
        // 地图保存成功
        const mapName = info.split(':')[1];
        const feedbackEl = document.getElementById('map-save-feedback');
        if (feedbackEl) {
            feedbackEl.textContent = `✅ 已保存: ${mapName}`;
            feedbackEl.style.color = 'var(--success)';
        }
        addLog(`✅ 地图保存成功: ${mapName}`);
        
        // 恢复保存按钮
        const saveBtn = document.getElementById('save-map-btn');
        if (saveBtn) {
            saveBtn.disabled = false;
            saveBtn.textContent = '💾 保存当前地图';
        }
        
        // 自动刷新地图列表
        setTimeout(() => {
            refreshMapList();
            // 清除反馈信息
            if (feedbackEl) {
                feedbackEl.textContent = '';
            }
        }, 3000);
    } else if (info.startsWith('map_save_failed:')) {
        // 地图保存失败
        const error = info.split(':')[1];
        const feedbackEl = document.getElementById('map-save-feedback');
        if (feedbackEl) {
            feedbackEl.textContent = `❌ 保存失败: ${error}`;
            feedbackEl.style.color = 'var(--danger)';
        }
        addLog(`❌ 地图保存失败: ${error}`);
        
        // 恢复保存按钮
        const saveBtn = document.getElementById('save-map-btn');
        if (saveBtn) {
            saveBtn.disabled = false;
            saveBtn.textContent = '💾 保存当前地图';
        }
        
        // 3秒后清除反馈信息
        setTimeout(() => {
            if (feedbackEl) {
                feedbackEl.textContent = '';
            }
        }, 3000);
    } else if (info.startsWith('slam_activating')) {
        // SLAM激活中
        stateElement.textContent = '⏳ SLAM启动中...';
        stateElement.style.color = 'var(--warning)';
        addLog('⏳ 正在激活SLAM节点...');
    } else if (info.startsWith('nav_activating')) {
        // 导航激活中
        stateElement.textContent = '⏳ 导航启动中...';
        stateElement.style.color = 'var(--warning)';
        addLog('⏳ 正在激活导航节点...');
    } else {
        addLog(`📡 导航信息: ${info}`);
    }
}

// 保存地图
function saveMap() {
    if (currentNavigationMode !== 1) {
        const feedbackEl = document.getElementById('map-save-feedback');
        if (feedbackEl) {
            feedbackEl.textContent = '⚠️ 仅在建图模式下可以保存地图';
            feedbackEl.style.color = 'var(--warning)';
        }
        addLog('⚠️ 仅在建图模式下可以保存地图');
        return;
    }
    
    const feedbackEl = document.getElementById('map-save-feedback');
    if (feedbackEl) {
        feedbackEl.textContent = '💾 保存中...';
        feedbackEl.style.color = 'var(--warning)';
    }
    
    // 禁用按钮防止重复点击
    const saveBtn = document.getElementById('save-map-btn');
    if (saveBtn) {
        saveBtn.disabled = true;
        saveBtn.textContent = '💾 保存中...';
    }
    
    sendWebSocketMessage({
        type: 'save_map'
    });
    addLog('📡 发送保存地图请求...');
    
    // 10秒后如果没有响应，恢复按钮
    setTimeout(() => {
        if (saveBtn) {
            saveBtn.disabled = false;
            saveBtn.textContent = '💾 保存当前地图';
        }
    }, 10000);
}

// 启动地图服务器
// 地图选择改变时通知后端
function onMapSelectChange() {
    const mapSelect = document.getElementById('map-select');
    const selectedMap = mapSelect.value;
    
    if (selectedMap) {
        sendWebSocketMessage({
            type: 'set_map_path',
            map_path: selectedMap
        });
        addLog(`📍 已选择导航地图: ${selectedMap.split('/').pop()}`);
    }
}

// 刷新地图列表
function refreshMapList() {
    sendWebSocketMessage({
        type: 'list_maps'
    });
    addLog('📡 刷新地图列表...');
}

// 更新地图列表（从服务器接收）
function updateMapList(maps) {
    availableMaps = maps;
    const mapSelect = document.getElementById('map-select');
    
    // 清空现有选项
    mapSelect.innerHTML = '<option value="">选择地图...</option>';
    
    // 添加地图选项
    maps.forEach(mapPath => {
        const option = document.createElement('option');
        const mapName = mapPath.split('/').pop().replace('.yaml', '');
        option.value = mapPath;
        option.textContent = mapName;
        mapSelect.appendChild(option);
    });
    
    addLog(`✅ 发现 ${maps.length} 个地图`);
}

// ==================== 导航功能 ====================

// 发送导航目标点
function sendNavigationGoal(x, y, theta = 0) {
    if (currentNavigationMode !== 2) {
        addLog('⚠️ 请先切换到导航模式');
        return;
    }
    
    sendWebSocketMessage({
        type: 'navigation_goal',
        x: x,
        y: y,
        theta: theta
    });
    addLog(`发送导航目标: (${x.toFixed(2)}, ${y.toFixed(2)})`);
}

// 取消导航
function cancelNavigation() {
    sendWebSocketMessage({
        type: 'cancel_navigation'
    });
    goalPose = null;
    path = [];
    addLog('取消导航');
}

// 请求地图更新
function requestMapUpdate() {
    sendWebSocketMessage({
        type: 'request_map'
    });
}

// 加载地图
function loadMap(mapName) {
    sendWebSocketMessage({
        type: 'load_map',
        name: mapName
    });
    addLog(`加载地图: ${mapName}`);
}

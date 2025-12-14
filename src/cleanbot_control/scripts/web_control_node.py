#!/usr/bin/env python3
"""
WebSocket控制节点 - 提供Web界面与机器人的交互
功能：
1. WebSocket服务器
2. HTTP静态文件服务
3. 机器人状态数据推送
4. 控制命令接收
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, UInt8MultiArray, String, UInt8
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import asyncio
import json
import threading
from aiohttp import web
import aiohttp
import os
from pathlib import Path
import time
import copy
from ament_index_python.packages import get_package_share_directory


class WebControlNode(Node):
    """Web控制节点"""
    
    def __init__(self):
        super().__init__('web_control_node')
        
        # 声明参数
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('ws_port', 8765)
        
        self.web_port = self.get_parameter('web_port').value
        self.ws_port = self.get_parameter('ws_port').value
        
        # 机器人状态数据
        self.robot_state = {
            'timestamp': 0.0,
            'usb_connected': False,
            'imu': {
                'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            },
            'wheels': {
                'left_position': 0.0,
                'left_velocity': 0.0,
                'right_position': 0.0,
                'right_velocity': 0.0
            },
            'sensors': {
                'bumper_left': 0,
                'bumper_right': 0,
                'ir_down0': 0,
                'ir_down1': 0,
                'ir_down2': 0,
                'fault_flags': 0,
                'heartbeat': 0,
                'dock_status': 0
            },
            'odometry': {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                'linear_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
            },
            'comm_quality': '',
            'system_status': ''
        }
        self.state_lock = threading.Lock()
        
        # WebSocket客户端连接列表
        self.ws_clients = set()
        
        # ROS订阅器
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data_raw', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.sensors_sub = self.create_subscription(
            UInt8MultiArray, 'sensors_status', self.sensors_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odometry/filtered', self.odom_callback, 10)
        self.usb_status_sub = self.create_subscription(
            Bool, 'usb_connected', self.usb_status_callback, 10)
        self.comm_quality_sub = self.create_subscription(
            String, 'comm_quality', self.comm_quality_callback, 10)
        self.system_status_sub = self.create_subscription(
            String, 'system_status', self.system_status_callback, 10)
        
        # ROS发布器 - 使用 TwistStamped（控制器监听 /diff_drive_controller/cmd_vel）
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.control_cmd_pub = self.create_publisher(
            UInt8MultiArray, 'control_command', 10)
        self.port_scan_pub = self.create_publisher(String, 'scan_connect_port', 10)
        
        # 导航模式发布器
        self.nav_mode_pub = self.create_publisher(UInt8, 'navigation/mode_cmd', 10)
        
        # 订阅导航模式状态
        self.nav_mode_status_sub = self.create_subscription(
            UInt8, 'navigation/mode_status', self.nav_mode_status_callback, 10)
        self.nav_info_sub = self.create_subscription(
            String, 'navigation/info', self.nav_info_callback, 10)
        
        # 导航相关服务客户端
        self.save_map_client = self.create_client(Empty, 'navigation/save_map')
        
        # 定时推送状态数据
        self.create_timer(0.1, self.broadcast_state)  # 10Hz
        
        self.get_logger().info(f'Web控制节点已启动 - HTTP端口:{self.web_port}')
        
        # 启动Web服务器（必须在最后，会阻塞）
        self.web_thread = threading.Thread(target=self.start_web_server, daemon=True)
        self.web_thread.start()
        
        # 等待Web服务器启动
        import time
        time.sleep(0.5)
        self.get_logger().info(f'Web服务器启动完成，访问 http://0.0.0.0:{self.web_port}')
    
    # ==================== ROS回调函数 ====================
    
    def imu_callback(self, msg: Imu):
        """IMU数据回调"""
        with self.state_lock:
            self.robot_state['imu'] = {
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z
                },
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                }
            }
    
    def joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        with self.state_lock:
            for i, name in enumerate(msg.name):
                if name == 'left_wheel_joint':
                    if i < len(msg.position):
                        self.robot_state['wheels']['left_position'] = msg.position[i]
                    if i < len(msg.velocity):
                        self.robot_state['wheels']['left_velocity'] = msg.velocity[i]
                elif name == 'right_wheel_joint':
                    if i < len(msg.position):
                        self.robot_state['wheels']['right_position'] = msg.position[i]
                    if i < len(msg.velocity):
                        self.robot_state['wheels']['right_velocity'] = msg.velocity[i]
    
    def sensors_callback(self, msg: UInt8MultiArray):
        """传感器状态回调"""
        if len(msg.data) >= 8:
            with self.state_lock:
                self.robot_state['sensors'] = {
                    'bumper_left': msg.data[0],
                    'bumper_right': msg.data[1],
                    'ir_down0': msg.data[2],
                    'ir_down1': msg.data[3],
                    'ir_down2': msg.data[4],
                    'fault_flags': msg.data[5],
                    'heartbeat': msg.data[6],
                    'dock_status': msg.data[7]
                }
    
    def odom_callback(self, msg: Odometry):
        """里程计回调"""
        with self.state_lock:
            self.robot_state['odometry'] = {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                },
                'linear_velocity': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular_velocity': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            }
    
    def usb_status_callback(self, msg: Bool):
        """USB连接状态回调"""
        with self.state_lock:
            self.robot_state['usb_connected'] = msg.data
    
    def comm_quality_callback(self, msg: String):
        """通讯质量回调"""
        with self.state_lock:
            self.robot_state['comm_quality'] = msg.data
    
    def system_status_callback(self, msg: String):
        """系统状态回调"""
        with self.state_lock:
            self.robot_state['system_status'] = msg.data
    
    def nav_mode_status_callback(self, msg: UInt8):
        """导航模式状态回调"""
        with self.state_lock:
            if 'navigation' not in self.robot_state:
                self.robot_state['navigation'] = {}
            self.robot_state['navigation']['mode'] = msg.data
    
    def nav_info_callback(self, msg: String):
        """导航信息回调"""
        # 通过WebSocket发送导航信息给前端
        if hasattr(self, 'loop') and self.loop:
            try:
                asyncio.run_coroutine_threadsafe(
                    self._send_nav_info(msg.data),
                    self.loop
                )
            except Exception as e:
                self.get_logger().error(f'发送导航信息失败: {e}')
    
    async def _send_nav_info(self, info: str):
        """异步发送导航信息到WebSocket客户端"""
        if not self.ws_clients:
            return
        
        message = json.dumps({
            'type': 'navigation_info',
            'message': info
        })
        
        for ws in list(self.ws_clients):
            try:
                if not ws.closed:
                    await ws.send_str(message)
            except Exception:
                pass
    
    # ==================== WebSocket相关 ====================
    
    def broadcast_state(self):
        """广播机器人状态到所有WebSocket客户端"""
        if not self.ws_clients:
            return
            
        if not hasattr(self, 'loop'):
            return
        
        with self.state_lock:
            try:
                state_copy = copy.deepcopy(self.robot_state)
                state_copy['timestamp'] = time.time()
            except Exception as e:
                self.get_logger().error(f'深拷贝状态数据失败: {e}')
                return
        
        # 在事件循环中发送消息
        try:
            asyncio.run_coroutine_threadsafe(
                self._broadcast_state_async(state_copy),
                self.loop
            )
        except Exception as e:
            self.get_logger().error(f'广播状态失败: {e}')
    
    async def _broadcast_state_async(self, state):
        """异步广播状态"""
        if not self.ws_clients:
            return
        
        # 序列化JSON
        try:
            message = json.dumps({
                'type': 'state_update',
                'data': state
            })
        except Exception as e:
            self.get_logger().error(f'JSON序列化失败: {e}')
            return
        
        # 发送给所有客户端
        disconnected_clients = set()
        
        for ws in self.ws_clients:
            try:
                if ws.closed:
                    disconnected_clients.add(ws)
                    continue
                await ws.send_str(message)
            except Exception as e:
                disconnected_clients.add(ws)
        
        # 移除断开的客户端
        if disconnected_clients:
            self.ws_clients -= disconnected_clients
    
    async def websocket_handler(self, request):
        """WebSocket连接处理"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self.ws_clients.add(ws)
        self.get_logger().info(f'WebSocket客户端连接，当前连接数: {len(self.ws_clients)}')
        
        # 发送欢迎消息
        try:
            welcome_msg = json.dumps({
                'type': 'info',
                'message': '连接成功！正在接收机器人状态数据...'
            })
            await ws.send_str(welcome_msg)
        except Exception as e:
            pass
        
        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    await self.handle_ws_message(msg.data, ws)
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    self.get_logger().error(f'WebSocket错误: {ws.exception()}')
        finally:
            self.ws_clients.discard(ws)
            self.get_logger().info(f'WebSocket客户端断开，当前连接数: {len(self.ws_clients)}')
        
        return ws
    
    async def handle_ws_message(self, message: str, ws):
        """处理WebSocket消息"""
        try:
            data = json.loads(message)
            cmd_type = data.get('type')
            
            if cmd_type == 'cmd_vel':
                # 速度控制命令
                linear = data.get('linear', 0.0)
                angular = data.get('angular', 0.0)
                
                # 发布 TwistStamped 消息
                # 注意：控制器配置 use_stamped_vel: true，监听 /diff_drive_controller/cmd_vel
                # 启动文件会将 cmd_vel 重映射到 /diff_drive_controller/cmd_vel
                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = 'base_footprint'
                twist_msg.twist.linear.x = float(linear)
                twist_msg.twist.angular.z = float(angular)
                self.cmd_vel_pub.publish(twist_msg)
                
            
            elif cmd_type == 'control_cmd':
                # 控制命令（模式、档位等）
                control_data = data.get('data', {})
                msg = UInt8MultiArray()
                msg.data = [
                    int(control_data.get('work_mode', 0)),
                    int(control_data.get('side_brush_left', 0)),
                    int(control_data.get('side_brush_right', 0)),
                    int(control_data.get('fan_level', 0)),
                    int(control_data.get('water_level', 0)),
                    int(control_data.get('need_ack', 0))
                ]
                self.control_cmd_pub.publish(msg)
            
            elif cmd_type == 'scan_ports':
                # 扫描串口
                msg = String()
                msg.data = 'scan'
                self.port_scan_pub.publish(msg)
            
            elif cmd_type == 'connect_port':
                # 连接串口
                port = data.get('port', '')
                msg = String()
                msg.data = port
                self.port_scan_pub.publish(msg)
            
            elif cmd_type == 'set_goal':
                # 设置目标点（预留接口，用于navigation2）
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                self.get_logger().info(f'收到目标点设置: ({x}, {y})')
                # TODO: 发布到navigation2的goal话题
                await ws.send_str(json.dumps({
                    'type': 'info',
                    'message': f'目标点已设置: ({x:.2f}, {y:.2f})'
                }))
            
            elif cmd_type == 'navigation_goal':
                # 发送导航目标（预留接口）
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                theta = data.get('theta', 0.0)
                self.get_logger().info(f'收到导航目标: ({x}, {y}, {theta})')
                # TODO: 使用nav2的action接口
                await ws.send_str(json.dumps({
                    'type': 'info',
                    'message': '导航目标已发送'
                }))
            
            elif cmd_type == 'cancel_navigation':
                # 取消导航（预留接口）
                self.get_logger().info('取消导航')
                # TODO: 取消navigation2的action
                await ws.send_str(json.dumps({
                    'type': 'info',
                    'message': '导航已取消'
                }))
            
            elif cmd_type == 'request_map':
                # 请求地图数据（预留接口）
                self.get_logger().info('请求地图更新')
                # TODO: 订阅并转发map话题
                await ws.send_str(json.dumps({
                    'type': 'map_update',
                    'data': {'message': '地图数据接口已预留'}
                }))
            
            elif cmd_type == 'navigation_mode':
                # 设置导航模式
                mode = data.get('mode', 0)
                msg = UInt8()
                msg.data = int(mode)
                self.nav_mode_pub.publish(msg)
                self.get_logger().info(f'切换导航模式: {mode}')
            
            elif cmd_type == 'save_map':
                # 保存地图
                self.get_logger().info('调用保存地图服务...')
                if self.save_map_client.wait_for_service(timeout_sec=1.0):
                    try:
                        request = Empty.Request()
                        future = self.save_map_client.call_async(request)
                        await ws.send_str(json.dumps({
                            'type': 'info',
                            'message': '正在保存地图...'
                        }))
                    except Exception as e:
                        self.get_logger().error(f'调用保存地图服务失败: {e}')
                        await ws.send_str(json.dumps({
                            'type': 'error',
                            'message': f'保存地图失败: {str(e)}'
                        }))
                else:
                    await ws.send_str(json.dumps({
                        'type': 'error',
                        'message': '保存地图服务不可用'
                    }))
            
            elif cmd_type == 'load_map':
                # 加载地图（预留接口）
                map_name = data.get('name', 'default_map')
                self.get_logger().info(f'加载地图: {map_name}')
                # TODO: 加载并发布地图
                await ws.send_str(json.dumps({
                    'type': 'info',
                    'message': f'地图加载接口已预留: {map_name}'
                }))
            
            else:
                await ws.send_str(json.dumps({
                    'type': 'error',
                    'message': f'未知命令类型: {cmd_type}'
                }))
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析失败: {str(e)}, 原始消息: {message[:100]}')
            await ws.send_str(json.dumps({
                'type': 'error',
                'message': f'JSON解析失败: {str(e)}'
            }))
        except Exception as e:
            self.get_logger().error(f'处理WebSocket消息失败: {str(e)}, 消息类型: {cmd_type if "cmd_type" in locals() else "未知"}')
            await ws.send_str(json.dumps({
                'type': 'error',
                'message': str(e)
            }))
    
    # ==================== HTTP服务器 ====================
    
    async def on_startup(self, app):
        """应用启动时的回调，获取正确的事件循环"""
        self.loop = asyncio.get_running_loop()
        self.get_logger().info(f'✅ 获取到事件循环: {self.loop}')
    
    def start_web_server(self):
        """启动Web服务器"""
        # 创建新的事件循环
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        app = web.Application()
        
        # 添加启动钩子来获取正确的事件循环
        app.on_startup.append(self.on_startup)
        
        # WebSocket路由
        app.router.add_get('/ws', self.websocket_handler)
        
        # 静态文件路由 - 使用正确的包路径
        try:
            package_share_dir = get_package_share_directory('cleanbot_control')
            static_dir = Path(package_share_dir) / 'web' / 'static'
            templates_dir = Path(package_share_dir) / 'web' / 'templates'
            self.get_logger().info(f'Web文件路径: {package_share_dir}')
        except Exception as e:
            self.get_logger().error(f'获取包路径失败: {e}')
            # 回退到开发模式路径
            package_dir = Path(__file__).parent.parent
            static_dir = package_dir / 'web' / 'static'
            templates_dir = package_dir / 'web' / 'templates'
        
        # 自定义静态文件处理器（支持符号链接和调试）
        async def serve_static(request):
            """自定义静态文件服务，支持符号链接"""
            filename = request.match_info.get('filename', '')
            filepath = static_dir / filename
            
            # 解析符号链接
            try:
                filepath = filepath.resolve()
            except Exception as e:
                self.get_logger().error(f'解析文件路径失败: {filepath}, 错误: {e}')
                return web.Response(text='File not found', status=404)
            
            if filepath.exists() and filepath.is_file():
                return web.FileResponse(filepath)
            else:
                self.get_logger().warn(f'静态文件不存在: {filepath}')
                return web.Response(text='File not found', status=404)
        
        if static_dir.exists():
            # 添加静态文件路由 - 使用自定义处理器
            app.router.add_get('/static/{filename:.+}', serve_static)
            self.get_logger().info(f'静态文件目录: {static_dir}')
            
            # 列出静态文件目录内容
            try:
                files = list(static_dir.iterdir())
                self.get_logger().info(f'静态文件列表: {[f.name for f in files]}')
            except Exception as e:
                self.get_logger().error(f'无法列出静态文件: {e}')
        else:
            self.get_logger().warn(f'静态文件目录不存在: {static_dir}')
        
        # 主页
        async def index(request):
            index_file = templates_dir / 'index.html'
            # 解析符号链接
            try:
                index_file = index_file.resolve()
            except Exception as e:
                self.get_logger().error(f'解析index.html路径失败: {e}')
                return web.Response(text='Index file not found', status=404)
            
            if index_file.exists():
                return web.FileResponse(index_file)
            else:
                error_msg = f'Web界面文件未找到: {index_file}'
                self.get_logger().error(error_msg)
                return web.Response(text=error_msg, status=404)
        
        app.router.add_get('/', index)
        
        # 启动服务器（禁用信号处理以支持线程运行）
        self.get_logger().info(f'启动Web服务器: http://0.0.0.0:{self.web_port}')
        self.get_logger().info(f'WebSocket地址: ws://0.0.0.0:{self.web_port}/ws')
        
        try:
            web.run_app(app, host='0.0.0.0', port=self.web_port, 
                        handle_signals=False, print=lambda x: None)
        except Exception as e:
            self.get_logger().error(f'Web服务器启动失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


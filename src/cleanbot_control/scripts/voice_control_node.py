#!/usr/bin/env python3
"""
语音控制节点 - 接收语音模块的串口指令，发布对应的控制命令
功能：
1. 串口通讯 - 接收语音模块的5字节帧
2. 指令解析 - 根据主码和子码匹配指令
3. 控制命令发布 - 向手动控制、导航模式、执行器发布命令
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt8MultiArray, UInt8
import serial
import threading
from typing import Optional
import time


class VoiceControlNode(Node):
    """语音控制节点"""
    
    # 导航模式定义
    NAV_MODE_MANUAL = 0
    NAV_MODE_MAPPING = 1
    NAV_MODE_NAVIGATION = 2
    
    # 手动控制模式定义
    CONTROL_MODE_JOYSTICK = 0
    CONTROL_MODE_ODOMETRY = 1
    
    def __init__(self):
        super().__init__('voice_control_node')
        
        # 声明参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('reconnect_interval', 2.0)
        
        # 速度控制参数（慢速/快速）
        self.declare_parameter('slow_linear_speed', 0.15)   # 慢速线速度 m/s
        self.declare_parameter('fast_linear_speed', 0.3)    # 快速线速度 m/s
        self.declare_parameter('slow_angular_speed', 0.5)   # 慢速角速度 rad/s
        self.declare_parameter('fast_angular_speed', 1.0)   # 快速角速度 rad/s
        self.declare_parameter('slow_turn_angular', 0.3)    # 慢速转弯角速度 rad/s
        self.declare_parameter('fast_turn_angular', 0.6)    # 快速转弯角速度 rad/s
        
        # 里程控制参数（小步/大步）
        self.declare_parameter('small_step_distance', 0.1)  # 小步距离 m
        self.declare_parameter('large_step_distance', 0.5)  # 大步距离 m
        self.declare_parameter('small_step_angle', 0.174)   # 小步角度 rad (约10度)
        self.declare_parameter('large_step_angle', 0.785)   # 大步角度 rad (约45度)
        self.declare_parameter('turn_around_angle', 3.14159)  # 向后转角度 rad (180度)
        
        # 获取参数
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.auto_reconnect = self.get_parameter('auto_reconnect').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        
        # 速度参数
        self.slow_linear = self.get_parameter('slow_linear_speed').value
        self.fast_linear = self.get_parameter('fast_linear_speed').value
        self.slow_angular = self.get_parameter('slow_angular_speed').value
        self.fast_angular = self.get_parameter('fast_angular_speed').value
        self.slow_turn_angular = self.get_parameter('slow_turn_angular').value
        self.fast_turn_angular = self.get_parameter('fast_turn_angular').value
        
        # 里程参数
        self.small_step_distance = self.get_parameter('small_step_distance').value
        self.large_step_distance = self.get_parameter('large_step_distance').value
        self.small_step_angle = self.get_parameter('small_step_angle').value
        self.large_step_angle = self.get_parameter('large_step_angle').value
        self.turn_around_angle = self.get_parameter('turn_around_angle').value
        
        # 串口相关
        self.serial_port: Optional[serial.Serial] = None
        self.rx_thread: Optional[threading.Thread] = None
        self.running = True
        
        # ROS发布器
        # 手动控制命令发布器（用于速度控制和里程控制）
        self.manual_control_pub = self.create_publisher(
            Float32MultiArray, 'manual_control_cmd', 10)
        
        # 执行器控制发布器（用于边刷、风机、水泵）
        self.control_cmd_pub = self.create_publisher(
            UInt8MultiArray, 'control_command', 10)
        
        # 导航模式发布器（用于切换手动/建图/导航模式）
        self.nav_mode_pub = self.create_publisher(
            UInt8, 'navigation/mode_cmd', 10)
        
        # 清扫模式发布器（用于切换待机/沿边/弓形/全屋模式）
        self.cleaning_mode_pub = self.create_publisher(
            UInt8, 'cleaning/mode_cmd', 10)
        
        # 当前执行器状态（用于记忆）
        self.current_actuators = {
            'work_mode': 0,
            'side_brush_left': 0,
            'side_brush_right': 0,
            'fan_level': 0,
            'water_level': 0
        }
        
        # 连接串口
        self.connect_serial()
        
        # 启动接收线程
        if self.serial_port:
            self.rx_thread = threading.Thread(target=self.rx_thread_func, daemon=True)
            self.rx_thread.start()
            self.get_logger().info('接收线程已启动')
        
        # 自动重连定时器
        if self.auto_reconnect:
            self.create_timer(self.reconnect_interval, self.reconnect_check)
        
        self.get_logger().info(f'语音控制节点已启动 - 串口: {self.port}, 波特率: {self.baudrate}')
    
    # ==================== 串口管理 ====================
    
    def connect_serial(self) -> bool:
        """连接串口"""
        if self.serial_port and self.serial_port.is_open:
            return True
        
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info(f'✅ 串口连接成功: {self.port}')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ 串口连接失败: {str(e)}')
            self.serial_port = None
            return False
    
    def disconnect_serial(self):
        """断开串口"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                self.get_logger().info('串口已断开')
            except Exception as e:
                self.get_logger().error(f'断开串口失败: {str(e)}')
        self.serial_port = None
    
    def reconnect_check(self):
        """定时检查并重连串口"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn('串口未连接，尝试重连...')
            self.connect_serial()
    
    def is_connected(self) -> bool:
        """检查串口连接状态"""
        return self.serial_port is not None and self.serial_port.is_open
    
    # ==================== 帧解析 ====================
    
    def parse_frame(self, frame: bytes) -> Optional[tuple]:
        """
        解析5字节帧
        返回: (main_code, sub_code) 或 None
        """
        if len(frame) != 5:
            return None
        
        # 验证帧头和帧尾
        if frame[0] != 0xAA or frame[1] != 0x55 or frame[4] != 0xFB:
            return None
        
        sub_code = frame[2]
        main_code = frame[3]
        
        return (main_code, sub_code)
    
    # ==================== 指令处理 ====================
    
    def handle_command(self, main_code: int, sub_code: int):
        """根据主码和子码处理指令"""
        
        # ========== 基础功能类 ==========
        if main_code == 0x00 and sub_code == 0x01:
            self.get_logger().info('🎤 欢迎语')
            return
        
        if main_code == 0x6F and sub_code == 0x02:
            self.get_logger().info('🎤 休息语')
            return
        
        if main_code == 0x00 and sub_code == 0x03:
            self.get_logger().info('🎤 你好双全')
            return
        
        if main_code == 0x00 and 0x04 <= sub_code <= 0x0A:
            volume_cmds = {
                0x04: '增大音量',
                0x05: '减小音量',
                0x06: '最大音量',
                0x07: '中等音量',
                0x08: '最小音量',
                0x09: '开启播报',
                0x0A: '关闭播报'
            }
            self.get_logger().info(f'🎤 {volume_cmds.get(sub_code, "未知音量指令")}')
            return
        
        # ========== 运动控制类 - 速度控制（遥控模式）==========
        if main_code == 0x01 and sub_code == 0x00:  # 慢速前进
            self.publish_joystick_cmd(self.slow_linear, 0.0)
            self.get_logger().info(f'🎤 慢速前进: {self.slow_linear} m/s')
            return
        
        if main_code == 0x02 and sub_code == 0x00:  # 快速前进
            self.publish_joystick_cmd(self.fast_linear, 0.0)
            self.get_logger().info(f'🎤 快速前进: {self.fast_linear} m/s')
            return
        
        if main_code == 0x03 and sub_code == 0x00:  # 慢速左旋
            self.publish_joystick_cmd(0.0, self.slow_angular)
            self.get_logger().info(f'🎤 慢速左旋: {self.slow_angular} rad/s')
            return
        
        if main_code == 0x04 and sub_code == 0x00:  # 快速左旋
            self.publish_joystick_cmd(0.0, self.fast_angular)
            self.get_logger().info(f'🎤 快速左旋: {self.fast_angular} rad/s')
            return
        
        if main_code == 0x05 and sub_code == 0x00:  # 慢速右旋
            self.publish_joystick_cmd(0.0, -self.slow_angular)
            self.get_logger().info(f'🎤 慢速右旋: {-self.slow_angular} rad/s')
            return
        
        if main_code == 0x06 and sub_code == 0x00:  # 快速右旋
            self.publish_joystick_cmd(0.0, -self.fast_angular)
            self.get_logger().info(f'🎤 快速右旋: {-self.fast_angular} rad/s')
            return
        
        if main_code == 0x07 and sub_code == 0x00:  # 慢速左转
            self.publish_joystick_cmd(self.slow_linear, self.slow_turn_angular)
            self.get_logger().info(f'🎤 慢速左转: linear={self.slow_linear}, angular={self.slow_turn_angular}')
            return
        
        if main_code == 0x08 and sub_code == 0x00:  # 快速左转
            self.publish_joystick_cmd(self.fast_linear, self.fast_turn_angular)
            self.get_logger().info(f'🎤 快速左转: linear={self.fast_linear}, angular={self.fast_turn_angular}')
            return
        
        if main_code == 0x09 and sub_code == 0x00:  # 慢速后退
            self.publish_joystick_cmd(-self.slow_linear, 0.0)
            self.get_logger().info(f'🎤 慢速后退: {-self.slow_linear} m/s')
            return
        
        if main_code == 0x0A and sub_code == 0x00:  # 快速后退
            self.publish_joystick_cmd(-self.fast_linear, 0.0)
            self.get_logger().info(f'🎤 快速后退: {-self.fast_linear} m/s')
            return
        
        if main_code == 0x0B and sub_code == 0x00:  # 慢速右转
            self.publish_joystick_cmd(self.slow_linear, -self.slow_turn_angular)
            self.get_logger().info(f'🎤 慢速右转: linear={self.slow_linear}, angular={-self.slow_turn_angular}')
            return
        
        if main_code == 0x0C and sub_code == 0x00:  # 快速右转
            self.publish_joystick_cmd(self.fast_linear, -self.fast_turn_angular)
            self.get_logger().info(f'🎤 快速右转: linear={self.fast_linear}, angular={-self.fast_turn_angular}')
            return
        
        # ========== 运动控制类 - 角度控制（里程模式）==========
        if main_code == 0x0D and sub_code == 0x00:  # 小步前进
            self.publish_odometry_cmd(self.small_step_distance, 0.0)
            self.get_logger().info(f'🎤 小步前进: {self.small_step_distance} m')
            return
        
        if main_code == 0x0E and sub_code == 0x00:  # 大步前进
            self.publish_odometry_cmd(self.large_step_distance, 0.0)
            self.get_logger().info(f'🎤 大步前进: {self.large_step_distance} m')
            return
        
        if main_code == 0x0F and sub_code == 0x00:  # 小步左旋
            self.publish_odometry_cmd(0.0, self.small_step_angle)
            self.get_logger().info(f'🎤 小步左旋: {self.small_step_angle} rad')
            return
        
        if main_code == 0x10 and sub_code == 0x00:  # 大步左旋
            self.publish_odometry_cmd(0.0, self.large_step_angle)
            self.get_logger().info(f'🎤 大步左旋: {self.large_step_angle} rad')
            return
        
        if main_code == 0x11 and sub_code == 0x00:  # 小步右旋
            self.publish_odometry_cmd(0.0, -self.small_step_angle)
            self.get_logger().info(f'🎤 小步右旋: {-self.small_step_angle} rad')
            return
        
        if main_code == 0x12 and sub_code == 0x00:  # 大步右旋
            self.publish_odometry_cmd(0.0, -self.large_step_angle)
            self.get_logger().info(f'🎤 大步右旋: {-self.large_step_angle} rad')
            return
        
        if main_code == 0x13 and sub_code == 0x00:  # 小步后退
            self.publish_odometry_cmd(-self.small_step_distance, 0.0)
            self.get_logger().info(f'🎤 小步后退: {-self.small_step_distance} m')
            return
        
        if main_code == 0x14 and sub_code == 0x00:  # 大步后退
            self.publish_odometry_cmd(-self.large_step_distance, 0.0)
            self.get_logger().info(f'🎤 大步后退: {-self.large_step_distance} m')
            return
        
        if main_code == 0x26 and sub_code == 0x00:  # 向后转
            self.publish_odometry_cmd(0.0, self.turn_around_angle)
            self.get_logger().info(f'🎤 向后转: {self.turn_around_angle} rad (180度)')
            return
        
        # ========== 附加功能类 - 执行器控制 ==========
        if main_code == 0x15 and sub_code == 0x00:  # 打开边刷
            self.set_actuator(side_brush_left=3, side_brush_right=3)
            self.get_logger().info('🎤 打开边刷: 左右均3档')
            return
        
        if main_code == 0x20 and sub_code == 0x00:  # 关闭边刷
            self.set_actuator(side_brush_left=0, side_brush_right=0)
            self.get_logger().info('🎤 关闭边刷')
            return
        
        if main_code == 0x21 and sub_code == 0x00:  # 打开吸尘
            self.set_actuator(fan_level=3)
            self.get_logger().info('🎤 打开吸尘: 3档')
            return
        
        if main_code == 0x16 and sub_code == 0x00:  # 强劲吸尘
            self.set_actuator(fan_level=5)
            self.get_logger().info('🎤 强劲吸尘: 5档')
            return
        
        if main_code == 0x17 and sub_code == 0x00:  # 关闭吸尘
            self.set_actuator(fan_level=0)
            self.get_logger().info('🎤 关闭吸尘')
            return
        
        if main_code == 0x18 and sub_code == 0x00:  # 打开洗地
            self.set_actuator(water_level=3)
            self.get_logger().info('🎤 打开洗地: 3档')
            return
        
        if main_code == 0x19 and sub_code == 0x00:  # 强劲洗地
            self.set_actuator(water_level=5)
            self.get_logger().info('🎤 强劲洗地: 5档')
            return
        
        if main_code == 0x1A and sub_code == 0x00:  # 关闭洗地
            self.set_actuator(water_level=0)
            self.get_logger().info('🎤 关闭洗地')
            return
        
        # ========== 模式切换类 ==========
        if main_code == 0x28 and sub_code == 0x00:  # 建图模式
            self.switch_navigation_mode(self.NAV_MODE_MAPPING)
            self.get_logger().info('🎤 切换到建图模式')
            return
        
        if main_code == 0x29 and sub_code == 0x00:  # 导航模式
            self.switch_navigation_mode(self.NAV_MODE_NAVIGATION)
            self.get_logger().info('🎤 切换到导航模式')
            return
        
        if main_code == 0x2A and sub_code == 0x00:  # 手动模式
            self.switch_navigation_mode(self.NAV_MODE_MANUAL)
            self.get_logger().info('🎤 切换到手动模式')
            return
        
        if main_code == 0x2B and sub_code == 0x00:  # 运动停止
            self.publish_joystick_cmd(0.0, 0.0)
            self.get_logger().info('🎤 运动停止')
            return
        
        # ========== 工作模式切换类（清扫模式）==========
        if main_code == 0x30 and sub_code == 0x00:  # 待机模式
            self.switch_cleaning_mode(0)
            self.get_logger().info('🎤 切换到待机模式')
            return
        
        if main_code == 0x31 and sub_code == 0x00:  # 自动全屋模式
            # 自动全屋需要先切换到导航模式
            self.switch_navigation_mode(self.NAV_MODE_NAVIGATION)
            self.switch_cleaning_mode(3)  # 清扫模式3 = 全屋覆盖
            self.get_logger().info('🎤 切换到自动全屋模式（导航模式 + 全屋覆盖）')
            return
        
        if main_code == 0x32 and sub_code == 0x00:  # 沿边模式
            # 沿边需要先切换到导航模式
            self.switch_navigation_mode(self.NAV_MODE_NAVIGATION)
            self.switch_cleaning_mode(1)  # 清扫模式1 = 沿边
            self.get_logger().info('🎤 切换到沿边模式（导航模式 + 沿边清扫）')
            return
        
        if main_code == 0x33 and sub_code == 0x00:  # 弓形模式
            # 弓形需要先切换到导航模式
            self.switch_navigation_mode(self.NAV_MODE_NAVIGATION)
            self.switch_cleaning_mode(2)  # 清扫模式2 = 弓形
            self.get_logger().info('🎤 切换到弓形模式（导航模式 + 弓形清扫）')
            return
        
        # 未识别的指令
        self.get_logger().warn(f'未识别的指令: 主码=0x{main_code:02X}, 子码=0x{sub_code:02X}')
    
    # ==================== ROS发布函数 ====================
    
    def publish_joystick_cmd(self, linear: float, angular: float):
        """
        发布遥控模式速度命令
        格式: [control_mode, linear_vel, angular_vel, target_distance, target_yaw]
        """
        msg = Float32MultiArray()
        msg.data = [
            float(self.CONTROL_MODE_JOYSTICK),  # 遥控模式
            float(linear),
            float(angular),
            0.0,  # 目标距离（遥控模式不使用）
            0.0   # 目标航向（遥控模式不使用）
        ]
        self.manual_control_pub.publish(msg)
    
    def publish_odometry_cmd(self, distance_increment: float, yaw_increment: float):
        """
        发布里程模式控制命令
        格式: [control_mode, linear_vel, angular_vel, target_distance, target_yaw]
        """
        msg = Float32MultiArray()
        msg.data = [
            float(self.CONTROL_MODE_ODOMETRY),  # 里程模式
            0.0,  # 线速度（里程模式不使用）
            0.0,  # 角速度（里程模式不使用）
            float(distance_increment),
            float(yaw_increment)
        ]
        self.manual_control_pub.publish(msg)
    
    def set_actuator(self, work_mode: int = None, side_brush_left: int = None, 
                     side_brush_right: int = None, fan_level: int = None, 
                     water_level: int = None):
        """
        设置执行器档位（只修改指定的参数，其他保持当前值）
        """
        if work_mode is not None:
            self.current_actuators['work_mode'] = work_mode
        if side_brush_left is not None:
            self.current_actuators['side_brush_left'] = side_brush_left
        if side_brush_right is not None:
            self.current_actuators['side_brush_right'] = side_brush_right
        if fan_level is not None:
            self.current_actuators['fan_level'] = fan_level
        if water_level is not None:
            self.current_actuators['water_level'] = water_level
        
        # 发布执行器命令
        msg = UInt8MultiArray()
        msg.data = [
            self.current_actuators['work_mode'],
            self.current_actuators['side_brush_left'],
            self.current_actuators['side_brush_right'],
            self.current_actuators['fan_level'],
            self.current_actuators['water_level'],
            0  # need_ack
        ]
        self.control_cmd_pub.publish(msg)
    
    def switch_navigation_mode(self, mode: int):
        """切换导航模式"""
        msg = UInt8()
        msg.data = mode
        self.nav_mode_pub.publish(msg)
    
    def switch_cleaning_mode(self, mode: int):
        """切换清扫模式"""
        msg = UInt8()
        msg.data = mode
        self.cleaning_mode_pub.publish(msg)
    
    # ==================== 接收线程 ====================
    
    def rx_thread_func(self):
        """串口接收线程"""
        self.get_logger().info('接收线程开始运行')
        
        frame_buffer = bytearray()
        
        while self.running:
            if not self.is_connected():
                time.sleep(0.1)
                continue
            
            try:
                # 读取数据
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    frame_buffer.extend(data)
                    
                    # 查找完整帧
                    while len(frame_buffer) >= 5:
                        # 查找帧头 0xAA 0x55
                        if frame_buffer[0] == 0xAA and frame_buffer[1] == 0x55:
                            # 检查是否有完整的5字节
                            if len(frame_buffer) >= 5:
                                # 提取帧
                                frame = bytes(frame_buffer[:5])
                                frame_buffer = frame_buffer[5:]
                                
                                # 解析帧
                                result = self.parse_frame(frame)
                                if result:
                                    main_code, sub_code = result
                                    # 处理指令
                                    self.handle_command(main_code, sub_code)
                                else:
                                    self.get_logger().warn(f'帧校验失败: {frame.hex()}')
                            else:
                                break  # 等待更多数据
                        else:
                            # 丢弃非法字节
                            frame_buffer.pop(0)
                
                time.sleep(0.01)  # 10ms
                
            except Exception as e:
                self.get_logger().error(f'接收数据失败: {str(e)}')
                time.sleep(0.1)
        
        self.get_logger().info('接收线程已退出')
    
    # ==================== 节点销毁 ====================
    
    def destroy_node(self):
        """销毁节点"""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        self.disconnect_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


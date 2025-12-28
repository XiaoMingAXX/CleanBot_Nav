#!/usr/bin/env python3
"""
USB通讯节点 - 负责与STM32的通讯协议实现
功能：
1. 数据收发（按照协议格式）
2. 通讯质量监测（帧率、丢包率、连接状态）
3. 自动重连机制
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32MultiArray, UInt8MultiArray, String
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class ProtocolFrame:
    """协议帧数据结构"""
    HEADER1 = 0x55
    HEADER2 = 0xAA
    VERSION = 0x01
    
    msg_id: int
    seq: int
    payload: bytes


class FrameParser:
    """帧解析器 - FSM状态机"""
    STATE_IDLE = 0
    STATE_HEADER1 = 1
    STATE_HEADER2 = 2
    STATE_VER = 3
    STATE_LEN = 4
    STATE_MSG_ID = 5
    STATE_SEQ = 6
    STATE_PAYLOAD = 7
    STATE_CRC = 8
    
    def __init__(self):
        self.state = self.STATE_IDLE
        self.buffer = bytearray()
        self.payload_len = 0
        self.msg_id = 0
        self.seq = 0
        self.payload = bytearray()
        self.crc_bytes = bytearray()
        
    def reset(self):
        """重置解析器状态"""
        self.state = self.STATE_IDLE
        self.buffer.clear()
        self.payload.clear()
        self.crc_bytes.clear()
        
    def parse_byte(self, byte: int) -> Optional[ProtocolFrame]:
        """
        解析单个字节
        返回: 完整帧或None
        """
        if self.state == self.STATE_IDLE:
            if byte == ProtocolFrame.HEADER1:
                self.buffer = bytearray([byte])
                self.state = self.STATE_HEADER1
                
        elif self.state == self.STATE_HEADER1:
            if byte == ProtocolFrame.HEADER2:
                self.buffer.append(byte)
                self.state = self.STATE_HEADER2
            else:
                self.reset()
                
        elif self.state == self.STATE_HEADER2:
            if byte == ProtocolFrame.VERSION:
                self.buffer.append(byte)
                self.state = self.STATE_VER
            else:
                self.reset()
                
        elif self.state == self.STATE_VER:
            self.buffer.append(byte)
            self.state = self.STATE_LEN
            
        elif self.state == self.STATE_LEN:
            self.buffer.append(byte)
            # 小端读取长度
            self.payload_len = struct.unpack('<H', self.buffer[-2:])[0]
            if self.payload_len > 1024:  # 防止异常长度
                self.reset()
            else:
                self.state = self.STATE_MSG_ID
                
        elif self.state == self.STATE_MSG_ID:
            self.msg_id = byte
            self.buffer.append(byte)
            self.state = self.STATE_SEQ
            
        elif self.state == self.STATE_SEQ:
            self.seq = byte
            self.buffer.append(byte)
            self.payload.clear()
            if self.payload_len > 0:
                self.state = self.STATE_PAYLOAD
            else:
                self.state = self.STATE_CRC
                
        elif self.state == self.STATE_PAYLOAD:
            self.payload.append(byte)
            self.buffer.append(byte)
            if len(self.payload) >= self.payload_len:
                self.state = self.STATE_CRC
                self.crc_bytes.clear()
                
        elif self.state == self.STATE_CRC:
            self.crc_bytes.append(byte)
            if len(self.crc_bytes) >= 2:
                # 验证CRC
                crc_received = struct.unpack('<H', bytes(self.crc_bytes))[0]
                crc_calculated = self.crc16_ccitt(self.buffer[2:])  # VER到PAYLOAD
                
                if crc_received == crc_calculated:
                    # 成功解析一帧
                    frame = ProtocolFrame(
                        msg_id=self.msg_id,
                        seq=self.seq,
                        payload=bytes(self.payload)
                    )
                    self.reset()
                    return frame
                else:
                    # CRC错误
                    self.reset()
                    
        return None
    
    @staticmethod
    def crc16_ccitt(data: bytes) -> int:
        """CRC-16-CCITT校验"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc


class FrameBuilder:
    """帧构建器"""
    def __init__(self):
        self.seq_counters = {}  # 每个MSG_ID独立的序列号计数器
        
    def build_frame(self, msg_id: int, payload: bytes) -> bytes:
        """构建完整的协议帧"""
        # 获取并更新序列号
        if msg_id not in self.seq_counters:
            self.seq_counters[msg_id] = 0
        seq = self.seq_counters[msg_id]
        self.seq_counters[msg_id] = (seq + 1) % 256
        
        # 构建帧
        frame = bytearray()
        frame.append(ProtocolFrame.HEADER1)
        frame.append(ProtocolFrame.HEADER2)
        frame.append(ProtocolFrame.VERSION)
        frame.extend(struct.pack('<H', len(payload)))  # 小端LEN
        frame.append(msg_id)
        frame.append(seq)
        frame.extend(payload)
        
        # 计算CRC (从VER到PAYLOAD)
        crc = FrameParser.crc16_ccitt(frame[2:])
        frame.extend(struct.pack('<H', crc))
        
        return bytes(frame)


class CommunicationMonitor:
    """通讯质量监控"""
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.frame_timestamps = {}  # msg_id -> deque of timestamps
        self.frame_seqs = {}  # msg_id -> last seq
        self.packet_loss_count = {}  # msg_id -> loss count
        
    def record_frame(self, msg_id: int, seq: int):
        """记录接收到的帧"""
        current_time = time.time()
        
        # 记录时间戳
        if msg_id not in self.frame_timestamps:
            self.frame_timestamps[msg_id] = deque(maxlen=self.window_size)
            self.packet_loss_count[msg_id] = 0
        self.frame_timestamps[msg_id].append(current_time)
        
        # 检测丢包
        if msg_id in self.frame_seqs:
            expected_seq = (self.frame_seqs[msg_id] + 1) % 256
            if seq != expected_seq:
                # 计算丢包数
                if seq > expected_seq:
                    loss = seq - expected_seq
                else:
                    loss = 256 - expected_seq + seq
                self.packet_loss_count[msg_id] += loss
        
        self.frame_seqs[msg_id] = seq
        
    def get_frame_rate(self, msg_id: int) -> float:
        """获取帧率(Hz)"""
        if msg_id not in self.frame_timestamps:
            return 0.0
        
        timestamps = list(self.frame_timestamps[msg_id])
        if len(timestamps) < 2:
            return 0.0
        
        time_span = timestamps[-1] - timestamps[0]
        if time_span > 0:
            return (len(timestamps) - 1) / time_span
        return 0.0
    
    def get_packet_loss_rate(self, msg_id: int) -> float:
        """获取丢包率"""
        if msg_id not in self.packet_loss_count:
            return 0.0
        
        if msg_id not in self.frame_timestamps or len(self.frame_timestamps[msg_id]) == 0:
            return 0.0
        
        total_expected = len(self.frame_timestamps[msg_id]) + self.packet_loss_count[msg_id]
        if total_expected > 0:
            return self.packet_loss_count[msg_id] / total_expected
        return 0.0


class USBCommunicationNode(Node):
    """USB通讯节点"""
    
    # 消息ID定义
    MSG_ID_CONTROL_CMD = 0x10
    MSG_ID_IMU = 0x20
    MSG_ID_WHEEL = 0x21
    MSG_ID_SENSORS_STATUS = 0x22
    MSG_ID_SYSTEM_STATUS = 0x23
    MSG_ID_ACK_REPLY = 0x24
    
    def __init__(self):
        super().__init__('usb_communication_node')
        
        # 声明参数
        self.declare_parameter('port', '/dev/mcu_serial')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('control_rate', 50.0)
        
        # 串口相关
        self.serial_port: Optional[serial.Serial] = None
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.auto_reconnect = self.get_parameter('auto_reconnect').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        
        # 协议相关
        self.frame_parser = FrameParser()
        self.frame_builder = FrameBuilder()
        self.comm_monitor = CommunicationMonitor()
        
        # 控制命令缓存
        self.control_cmd = {
            'left_wheel_speed': 0.0,
            'right_wheel_speed': 0.0,
            'work_mode': 0,
            'side_brush_left': 0,
            'side_brush_right': 0,
            'fan_level': 0,
            'water_level': 0,
            'need_ack': False
        }
        self.control_lock = threading.Lock()
        
        # ROS发布器
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sensors_pub = self.create_publisher(UInt8MultiArray, 'sensors_status', 10)
        self.system_status_pub = self.create_publisher(String, 'system_status', 10)
        self.connection_status_pub = self.create_publisher(Bool, 'usb_connected', 10)
        self.comm_quality_pub = self.create_publisher(String, 'comm_quality', 10)
        
        # ROS订阅器
        # 订阅轮速命令（来自硬件接口，单位：m/s）
        self.wheel_speed_cmd_sub = self.create_subscription(
            Float32MultiArray, 'wheel_speed_cmd', self.wheel_speed_cmd_callback, 10)
        # 订阅控制命令（档位控制）
        self.control_cmd_sub = self.create_subscription(
            UInt8MultiArray, 'control_command', self.control_cmd_callback, 10)
        self.port_scan_sub = self.create_subscription(
            String, 'scan_connect_port', self.scan_connect_callback, 10)
        
        # 定时器
        control_rate = self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(
            1.0 / control_rate, self.control_timer_callback)
        self.monitor_timer = self.create_timer(1.0, self.monitor_timer_callback)
        
        # 线程
        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_thread_func, daemon=True)
        self.rx_thread.start()
        
        # 初始连接
        if not self.connect_serial():
            self.get_logger().warn(f'初始连接失败，将在后台尝试自动重连')
        
        self.get_logger().info('USB通讯节点已启动')
    
    def connect_serial(self, port: Optional[str] = None) -> bool:
        """连接串口"""
        if port:
            self.port = port
        
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.1
            )
            self.get_logger().info(f'成功连接到 {self.port}')
            return True
        except Exception as e:
            self.get_logger().warn(f'无法连接到 {self.port}: {str(e)}')
            return False
    
    def scan_connect_callback(self, msg: String):
        """扫描并连接串口"""
        if msg.data == 'scan':
            # 扫描可用串口
            ports = serial.tools.list_ports.comports()
            port_list = [port.device for port in ports]
            self.get_logger().info(f'可用串口: {port_list}')
            # 发布可用串口列表
            status_msg = String()
            status_msg.data = f'available_ports:{",".join(port_list)}'
            self.system_status_pub.publish(status_msg)
        else:
            # 尝试连接指定串口
            if self.connect_serial(msg.data):
                status_msg = String()
                status_msg.data = f'connected:{msg.data}'
                self.system_status_pub.publish(status_msg)
    
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.serial_port is not None and self.serial_port.is_open
    
    def wheel_speed_cmd_callback(self, msg: Float32MultiArray):
        """轮速命令回调 - 直接接收左右轮速度（m/s）"""
        if len(msg.data) >= 2:
            with self.control_lock:
                self.control_cmd['left_wheel_speed'] = msg.data[0]
                self.control_cmd['right_wheel_speed'] = msg.data[1]
    
    def control_cmd_callback(self, msg: UInt8MultiArray):
        """控制命令回调 - 档位控制"""
        # 格式: [work_mode, side_brush_left, side_brush_right, fan_level, water_level, need_ack]
        if len(msg.data) >= 6:
            with self.control_lock:
                self.control_cmd['work_mode'] = msg.data[0]
                self.control_cmd['side_brush_left'] = msg.data[1]
                self.control_cmd['side_brush_right'] = msg.data[2]
                self.control_cmd['fan_level'] = msg.data[3]
                self.control_cmd['water_level'] = msg.data[4]
                self.control_cmd['need_ack'] = bool(msg.data[5])
    
    def control_timer_callback(self):
        """定时发送控制命令"""
        if not self.is_connected():
            return
        
        with self.control_lock:
            # 构建payload (14字节)
            payload = struct.pack(
                '<ffBBBBBBB',
                self.control_cmd['left_wheel_speed'],
                self.control_cmd['right_wheel_speed'],
                self.control_cmd['work_mode'],
                self.control_cmd['side_brush_left'],
                self.control_cmd['side_brush_right'],
                self.control_cmd['fan_level'],
                self.control_cmd['water_level'],
                1 if self.control_cmd['need_ack'] else 0,
                0  # reserved
            )
        
        frame = self.frame_builder.build_frame(self.MSG_ID_CONTROL_CMD, payload)
        
        try:
            self.serial_port.write(frame)
        except Exception as e:
            self.get_logger().error(f'发送控制命令失败: {str(e)}')
    
    def rx_thread_func(self):
        """接收线程"""
        reconnect_timer = 0
        
        while self.running:
            # 检查连接状态
            if not self.is_connected():
                self.connection_status_pub.publish(Bool(data=False))
                
                if self.auto_reconnect:
                    current_time = time.time()
                    if current_time - reconnect_timer > self.reconnect_interval:
                        self.get_logger().info('尝试自动重连...')
                        self.connect_serial()
                        reconnect_timer = current_time
                
                time.sleep(0.1)
                continue
            
            self.connection_status_pub.publish(Bool(data=True))
            
            # 读取数据
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    for byte in data:
                        frame = self.frame_parser.parse_byte(byte)
                        if frame:
                            self.process_frame(frame)
                else:
                    time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f'读取数据失败: {str(e)}')
                if self.serial_port:
                    self.serial_port.close()
                time.sleep(0.1)
    
    def process_frame(self, frame: ProtocolFrame):
        """处理接收到的帧"""
        # 记录到监控器
        self.comm_monitor.record_frame(frame.msg_id, frame.seq)
        
        try:
            if frame.msg_id == self.MSG_ID_IMU:
                self.process_imu_frame(frame.payload)
            elif frame.msg_id == self.MSG_ID_WHEEL:
                self.process_wheel_frame(frame.payload)
            elif frame.msg_id == self.MSG_ID_SENSORS_STATUS:
                self.process_sensors_frame(frame.payload)
            elif frame.msg_id == self.MSG_ID_SYSTEM_STATUS:
                self.process_system_status_frame(frame.payload)
            elif frame.msg_id == self.MSG_ID_ACK_REPLY:
                self.process_ack_frame(frame.payload)
        except Exception as e:
            self.get_logger().error(f'处理帧失败 (ID={frame.msg_id:02x}): {str(e)}')
    
    def process_imu_frame(self, payload: bytes):
        """处理IMU数据帧 (36字节)"""
        if len(payload) < 36:
            return
        
        data = struct.unpack('<fffffffff', payload)
        ax, ay, az, gx, gy, gz, roll, pitch, yaw = data
        
        # 发布IMU消息
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # 线性加速度
        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)
        
        # 角速度
        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)
        
        # 四元数 (从欧拉角转换)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        imu_msg.orientation.w = float(cr * cp * cy + sr * sp * sy)
        imu_msg.orientation.x = float(sr * cp * cy - cr * sp * sy)
        imu_msg.orientation.y = float(cr * sp * cy + sr * cp * sy)
        imu_msg.orientation.z = float(cr * cp * sy - sr * sp * cy)
        
        self.imu_pub.publish(imu_msg)
    
    def process_wheel_frame(self, payload: bytes):
        """处理轮速数据帧 (16字节)"""
        if len(payload) < 16:
            return
        
        data = struct.unpack('<ffff', payload)
        left_angle_deg, left_speed_mps, right_angle_deg, right_speed_mps = data
        
        # 轮子半径（应与控制器配置一致）
        wheel_radius = 0.032  # m
        
        # 转换角度为弧度
        left_angle_rad = np.deg2rad(left_angle_deg)
        right_angle_rad = np.deg2rad(right_angle_deg)
        
        # 转换速度：m/s → rad/s，公式：ω = v / r
        left_speed_rad_s = left_speed_mps / wheel_radius
        right_speed_rad_s = right_speed_mps / wheel_radius
        
        # 发布JointState消息
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_msg.position = [float(left_angle_rad), float(right_angle_rad)]
        joint_msg.velocity = [float(left_speed_rad_s), float(right_speed_rad_s)]
        
        self.joint_state_pub.publish(joint_msg)
    
    def process_sensors_frame(self, payload: bytes):
        """处理传感器状态帧"""
        if len(payload) < 9:
            return
        
        data = struct.unpack('<BBBBBBBBB', payload)
        
        # 发布传感器状态
        sensor_msg = UInt8MultiArray()
        sensor_msg.data = list(data)
        self.sensors_pub.publish(sensor_msg)
    
    def process_system_status_frame(self, payload: bytes):
        """处理系统状态帧"""
        # 根据实际payload定义解析
        status_msg = String()
        status_msg.data = f'system_status:{payload.hex()}'
        self.system_status_pub.publish(status_msg)
    
    def process_ack_frame(self, payload: bytes):
        """处理ACK回复帧"""
        if len(payload) < 2:
            return
        
        cmd_id, status = struct.unpack('<BB', payload[:2])
        self.get_logger().info(f'收到ACK: CMD_ID={cmd_id:02x}, STATUS={status}')
    
    def monitor_timer_callback(self):
        """监控定时器 - 发布通讯质量数据"""
        if not self.is_connected():
            return
        
        # 收集统计信息
        quality_info = []
        for msg_id in [self.MSG_ID_IMU, self.MSG_ID_WHEEL, self.MSG_ID_SENSORS_STATUS]:
            frame_rate = self.comm_monitor.get_frame_rate(msg_id)
            loss_rate = self.comm_monitor.get_packet_loss_rate(msg_id)
            quality_info.append(f'0x{msg_id:02x}:{frame_rate:.1f}Hz,丢包{loss_rate*100:.1f}%')
        
        quality_msg = String()
        quality_msg.data = ' | '.join(quality_info)
        self.comm_quality_pub.publish(quality_msg)
    
    def destroy_node(self):
        """节点销毁"""
        self.running = False
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBCommunicationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



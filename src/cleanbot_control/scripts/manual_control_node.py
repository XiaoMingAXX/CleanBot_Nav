#!/usr/bin/env python3
"""
手动控制节点 - 实现遥控和里程闭环控制
功能：
1. 遥控模式：直接转发前端速度命令（带红外安全限制）
2. 里程模式：基于目标里程和航向的PID闭环控制
3. 路程解算：基于里程计数据计算累积路程
4. 红外安全融合：前/左/右红外触发时限制速度
5. 导航模式适配：仅在手动/建图模式下发布速度
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt8MultiArray, UInt8, Float32MultiArray
import math
import numpy as np
from collections import deque


class PIDController:
    """PID控制器"""
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        
    def compute(self, setpoint, measurement, current_time):
        """计算PID输出"""
        error = setpoint - measurement
        
        # 时间差
        if self.last_time is None:
            dt = 0.0
        else:
            dt = current_time - self.last_time
        
        self.last_time = current_time
        
        if dt <= 0.0:
            return 0.0
        
        # 比例项
        p_term = self.kp * error
        
        # 积分项
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # 微分项
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative
        
        # 总输出
        output = p_term + i_term + d_term
        
        # 限幅
        output = max(-self.output_limit, min(self.output_limit, output))
        
        # 抗积分饱和
        if abs(output) >= self.output_limit:
            self.integral -= error * dt  # 回退积分
        
        self.last_error = error
        
        return output
    
    def reset(self):
        """重置PID状态"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None


class ManualControlNode(Node):
    """手动控制节点"""
    
    # 控制模式定义
    CONTROL_MODE_JOYSTICK = 0  # 遥控模式
    CONTROL_MODE_ODOMETRY = 1  # 里程闭环模式
    
    # 导航模式定义（需与navigation_mode_manager一致）
    NAV_MODE_MANUAL = 0
    NAV_MODE_MAPPING = 1
    NAV_MODE_NAVIGATION = 2
    
    def __init__(self):
        super().__init__('manual_control_node')
        
        # ==================== 参数声明 ====================
        # 控制频率
        self.declare_parameter('control_frequency', 50.0)
        
        # PID参数 - 路程控制
        self.declare_parameter('distance_pid.kp', 0.5)
        self.declare_parameter('distance_pid.ki', 0.0)
        self.declare_parameter('distance_pid.kd', 0.1)
        self.declare_parameter('distance_pid.output_limit', 0.3)  # m/s
        
        # PID参数 - 航向控制
        self.declare_parameter('yaw_pid.kp', 1.0)
        self.declare_parameter('yaw_pid.ki', 0.0)
        self.declare_parameter('yaw_pid.kd', 0.2)
        self.declare_parameter('yaw_pid.output_limit', 1.0)  # rad/s
        
        # 速度限幅
        self.declare_parameter('max_linear_velocity', 0.3)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        
        # 里程模式容差
        self.declare_parameter('distance_tolerance', 0.01)  # m
        self.declare_parameter('yaw_tolerance', 0.05)  # rad
        
        # 红外传感器安全速度限制
        self.declare_parameter('ir_triggered_max_linear_velocity', 0.0)  # 红外触发时最大线速度
        
        # ==================== 获取参数 ====================
        control_freq = self.get_parameter('control_frequency').value
        
        # 创建PID控制器
        self.distance_pid = PIDController(
            kp=self.get_parameter('distance_pid.kp').value,
            ki=self.get_parameter('distance_pid.ki').value,
            kd=self.get_parameter('distance_pid.kd').value,
            output_limit=self.get_parameter('distance_pid.output_limit').value
        )
        
        self.yaw_pid = PIDController(
            kp=self.get_parameter('yaw_pid.kp').value,
            ki=self.get_parameter('yaw_pid.ki').value,
            kd=self.get_parameter('yaw_pid.kd').value,
            output_limit=self.get_parameter('yaw_pid.output_limit').value
        )
        
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.ir_max_linear_vel = self.get_parameter('ir_triggered_max_linear_velocity').value
        
        # ==================== 状态变量 ====================
        # 控制模式
        self.control_mode = self.CONTROL_MODE_JOYSTICK
        
        # 导航模式（从navigation_mode_manager订阅）
        self.navigation_mode = self.NAV_MODE_MANUAL
        
        # 里程计相关
        self.odom_initialized = False
        self.longitudinal_displacement = 0.0  # 纵向位移（前进为正，后退为负）
        self.last_odom_time = None  # 上次里程计时间
        self.accumulated_distance = 0.0  # 累积路程（仅用于显示）
        
        # IMU相关
        self.imu_initialized = False
        self.current_yaw = 0.0  # 当前航向角
        
        # 红外传感器状态
        self.ir_right = 0   # ir_down0
        self.ir_front = 0   # ir_down1
        self.ir_left = 0    # ir_down2
        
        # 前端遥控命令
        self.joystick_linear = 0.0
        self.joystick_angular = 0.0
        
        # 里程模式目标
        self.target_displacement = 0.0  # 目标纵向位移
        self.target_yaw = 0.0  # 目标航向角
        
        # ==================== ROS订阅器 ====================
        # 订阅里程计数据
        self.odom_sub = self.create_subscription(
            Odometry, 'odometry/filtered', self.odom_callback, 10)
        
        # 订阅IMU数据
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data_raw', self.imu_callback, 10)
        
        # 订阅红外传感器数据
        self.sensors_sub = self.create_subscription(
            UInt8MultiArray, 'sensors_status', self.sensors_callback, 10)
        
        # 订阅导航模式状态
        self.nav_mode_sub = self.create_subscription(
            UInt8, 'navigation/mode_status', self.nav_mode_callback, 10)
        
        # 订阅前端控制命令（新增话题）
        self.manual_cmd_sub = self.create_subscription(
            Float32MultiArray, 'manual_control_cmd', self.manual_cmd_callback, 10)
        
        # ==================== ROS发布器 ====================
        # 发布速度命令到差速控制器
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        
        # 发布累积路程反馈给前端
        self.distance_feedback_pub = self.create_publisher(
            Float32MultiArray, 'distance_feedback', 10)
        
        # ==================== 定时器 ====================
        self.control_timer = self.create_timer(
            1.0 / control_freq, self.control_loop)
        
        # 定时发布反馈数据（10Hz）
        self.feedback_timer = self.create_timer(0.1, self.publish_feedback)
        
        self.get_logger().info('手动控制节点已启动')
        self.get_logger().info(f'控制频率: {control_freq} Hz')
        self.get_logger().info(f'速度限制: linear={self.max_linear_vel} m/s, angular={self.max_angular_vel} rad/s')
    
    # ==================== 回调函数 ====================
    
    def odom_callback(self, msg: Odometry):
        """里程计回调 - 通过线速度积分得到纵向位移"""
        # 获取线速度（机器人坐标系下的X方向速度）
        vx = msg.twist.twist.linear.x  # m/s
        
        # 获取当前时间
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        if not self.odom_initialized:
            self.last_odom_time = current_time
            self.odom_initialized = True
            return
        
        # 计算时间差
        dt = current_time - self.last_odom_time
        
        if dt > 0 and dt < 1.0:  # 防止异常时间差
            # 通过线速度积分得到位移增量
            # 位移 = ∫ vx · dt
            # 前进（vx>0）增加位移，后退（vx<0）减少位移
            delta_displacement = vx * dt
            self.longitudinal_displacement += delta_displacement
            
            # 同时计算累积路程（仅用于显示，取绝对值累加）
            self.accumulated_distance += abs(delta_displacement)
        
        # 更新上次时间
        self.last_odom_time = current_time
    
    def imu_callback(self, msg: Imu):
        """IMU回调 - 提取航向角"""
        # 从四元数提取yaw角
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if not self.imu_initialized:
            self.imu_initialized = True
    
    def sensors_callback(self, msg: UInt8MultiArray):
        """传感器回调 - 红外状态"""
        if len(msg.data) >= 5:
            self.ir_right = msg.data[2]  # ir_down0
            self.ir_front = msg.data[3]  # ir_down1
            self.ir_left = msg.data[4]   # ir_down2
    
    def nav_mode_callback(self, msg: UInt8):
        """导航模式回调"""
        self.navigation_mode = msg.data
    
    def manual_cmd_callback(self, msg: Float32MultiArray):
        """
        前端控制命令回调
        格式: [control_mode, linear_vel, angular_vel, target_displacement, target_yaw]
        - control_mode: 0=遥控模式, 1=里程模式
        - linear_vel: 遥控模式下的线速度 (m/s)
        - angular_vel: 遥控模式下的角速度 (rad/s)
        - target_displacement: 里程模式下的目标纵向位移增量 (m)
        - target_yaw: 里程模式下的目标航向增量 (rad)
        """
        if len(msg.data) < 5:
            return
        
        new_mode = int(msg.data[0])
        
        # 模式切换处理
        if new_mode != self.control_mode:
            self.control_mode = new_mode
            self.get_logger().info(f'切换控制模式: {"遥控" if new_mode == 0 else "里程"}')
            
            # 切换到里程模式时，初始化目标为当前值（避免意外）
            if self.control_mode == self.CONTROL_MODE_ODOMETRY:
                self.target_displacement = self.longitudinal_displacement
                self.target_yaw = self.current_yaw
                self.distance_pid.reset()
                self.yaw_pid.reset()
                self.get_logger().info(f'里程模式初始化: displacement={self.target_displacement:.3f}m, yaw={self.target_yaw:.3f}rad')
        
        # 更新命令
        if self.control_mode == self.CONTROL_MODE_JOYSTICK:
            # 遥控模式
            self.joystick_linear = float(msg.data[1])
            self.joystick_angular = float(msg.data[2])
        
        elif self.control_mode == self.CONTROL_MODE_ODOMETRY:
            # 里程模式 - 增量目标
            displacement_increment = float(msg.data[3])
            yaw_increment = float(msg.data[4])
            
            # 只有增量非零时才更新目标
            if abs(displacement_increment) > 1e-6:
                self.target_displacement = self.longitudinal_displacement + displacement_increment
                self.get_logger().info(f'设置目标位移: {self.target_displacement:.3f}m (增量: {displacement_increment:.3f}m)')
            
            if abs(yaw_increment) > 1e-6:
                self.target_yaw = self.normalize_angle(self.current_yaw + yaw_increment)
                self.get_logger().info(f'设置目标航向: {self.target_yaw:.3f}rad (增量: {yaw_increment:.3f}rad)')
    
    # ==================== 控制循环 ====================
    
    def control_loop(self):
        """主控制循环"""
        # 检查导航模式 - 只在手动/建图模式下输出速度
        if self.navigation_mode == self.NAV_MODE_NAVIGATION:
            # 导航模式下不输出，避免干扰导航组件
            return
        
        # 根据控制模式计算速度
        if self.control_mode == self.CONTROL_MODE_JOYSTICK:
            vx, wz = self.compute_joystick_control()
        elif self.control_mode == self.CONTROL_MODE_ODOMETRY:
            vx, wz = self.compute_odometry_control()
        else:
            vx, wz = 0.0, 0.0
        
        # 融合红外数据进行安全限制
        vx, wz = self.apply_ir_safety(vx, wz)
        
        # 发布速度命令
        self.publish_velocity(vx, wz)
    
    def compute_joystick_control(self):
        """遥控模式 - 直接转发前端速度"""
        vx = self.joystick_linear
        wz = self.joystick_angular
        
        # 限幅
        vx = max(-self.max_linear_vel, min(self.max_linear_vel, vx))
        wz = max(-self.max_angular_vel, min(self.max_angular_vel, wz))
        
        return vx, wz
    
    def compute_odometry_control(self):
        """里程模式 - PID闭环控制"""
        if not self.odom_initialized or not self.imu_initialized:
            return 0.0, 0.0
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 纵向位移PID控制
        vx = self.distance_pid.compute(
            self.target_displacement, self.longitudinal_displacement, current_time)
        
        # 航向PID控制（处理角度归一化）
        yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
        wz = self.yaw_pid.compute(
            0.0, -yaw_error, current_time)  # setpoint=0, measurement=-error
        
        # 检查是否到达目标
        displacement_error = abs(self.target_displacement - self.longitudinal_displacement)
        yaw_error_abs = abs(yaw_error)
        
        if displacement_error < self.distance_tolerance and yaw_error_abs < self.yaw_tolerance:
            # 到达目标，停止并将目标设为当前值
            vx, wz = 0.0, 0.0
            self.target_displacement = self.longitudinal_displacement
            self.target_yaw = self.current_yaw
        
        # 限幅
        vx = max(-self.max_linear_vel, min(self.max_linear_vel, vx))
        wz = max(-self.max_angular_vel, min(self.max_angular_vel, wz))
        
        return vx, wz
    
    def apply_ir_safety(self, vx, wz):
        """融合红外数据进行安全限制"""
        # 前方红外触发：限制线速度 <= 0
        if self.ir_front > 0:
            if vx > self.ir_max_linear_vel:
                vx = self.ir_max_linear_vel
            # 如果当前在里程模式，修改目标为当前值
            if self.control_mode == self.CONTROL_MODE_ODOMETRY:
                self.target_displacement = self.longitudinal_displacement
                self.distance_pid.reset()
        
        # 左边红外触发：限制角速度 <= 0（不允许左转）
        if self.ir_left > 0:
            if wz > 0:
                wz = 0.0
            if self.control_mode == self.CONTROL_MODE_ODOMETRY:
                self.target_yaw = self.current_yaw
                self.yaw_pid.reset()
        
        # 右边红外触发：限制角速度 >= 0（不允许右转）
        if self.ir_right > 0:
            if wz < 0:
                wz = 0.0
            if self.control_mode == self.CONTROL_MODE_ODOMETRY:
                self.target_yaw = self.current_yaw
                self.yaw_pid.reset()
        
        return vx, wz
    
    def publish_velocity(self, vx, wz):
        """发布速度命令"""
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_footprint'
        cmd_msg.twist.linear.x = vx
        cmd_msg.twist.angular.z = wz
        self.cmd_vel_pub.publish(cmd_msg)
    
    def publish_feedback(self):
        """发布反馈数据给前端"""
        feedback_msg = Float32MultiArray()
        feedback_msg.data = [
            float(self.longitudinal_displacement),  # 纵向位移
            float(self.current_yaw),  # 当前航向
            float(self.control_mode),  # 当前控制模式
            float(self.navigation_mode),  # 当前导航模式
            float(self.accumulated_distance)  # 累积路程（仅用于显示）
        ]
        self.distance_feedback_pub.publish(feedback_msg)
    
    # ==================== 工具函数 ====================
    
    @staticmethod
    def normalize_angle(angle):
        """角度归一化到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


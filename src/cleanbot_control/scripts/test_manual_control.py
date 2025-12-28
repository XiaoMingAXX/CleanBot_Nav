#!/usr/bin/env python3
"""
手动控制测试脚本
用于测试manual_control_node的功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class ManualControlTester(Node):
    """手动控制测试节点"""
    
    def __init__(self):
        super().__init__('manual_control_tester')
        
        # 发布器
        self.cmd_pub = self.create_publisher(
            Float32MultiArray, 'manual_control_cmd', 10)
        
        # 订阅反馈
        self.feedback_sub = self.create_subscription(
            Float32MultiArray, 'distance_feedback', self.feedback_callback, 10)
        
        self.last_feedback = None
        
        self.get_logger().info('手动控制测试节点已启动')
    
    def feedback_callback(self, msg: Float32MultiArray):
        """反馈回调"""
        if len(msg.data) >= 4:
            self.last_feedback = {
                'distance': msg.data[0],
                'yaw': msg.data[1],
                'control_mode': int(msg.data[2]),
                'nav_mode': int(msg.data[3])
            }
    
    def send_joystick_cmd(self, linear, angular):
        """发送遥控命令"""
        cmd = Float32MultiArray()
        cmd.data = [0.0, float(linear), float(angular), 0.0, 0.0]
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'发送遥控命令: vx={linear}, wz={angular}')
    
    def send_odometry_cmd(self, distance, yaw):
        """发送里程控制命令"""
        cmd = Float32MultiArray()
        cmd.data = [1.0, 0.0, 0.0, float(distance), float(yaw)]
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'发送里程命令: distance={distance}, yaw={yaw}')
    
    def print_feedback(self):
        """打印反馈信息"""
        if self.last_feedback:
            self.get_logger().info(
                f'反馈 - 累积路程: {self.last_feedback["distance"]:.3f}m, '
                f'航向: {self.last_feedback["yaw"]:.3f}rad, '
                f'控制模式: {self.last_feedback["control_mode"]}, '
                f'导航模式: {self.last_feedback["nav_mode"]}'
            )
        else:
            self.get_logger().warn('暂未收到反馈数据')


def main(args=None):
    rclpy.init(args=args)
    tester = ManualControlTester()
    
    # 等待连接
    print('\n等待连接...')
    time.sleep(2)
    
    print('\n======== 手动控制测试菜单 ========')
    print('1. 遥控模式 - 前进')
    print('2. 遥控模式 - 后退')
    print('3. 遥控模式 - 左转')
    print('4. 遥控模式 - 右转')
    print('5. 遥控模式 - 停止')
    print('6. 里程模式 - 前进1米')
    print('7. 里程模式 - 后退0.5米')
    print('8. 里程模式 - 左转90度')
    print('9. 里程模式 - 右转90度')
    print('0. 查看反馈')
    print('q. 退出')
    print('==================================\n')
    
    try:
        while rclpy.ok():
            # 处理ROS消息
            rclpy.spin_once(tester, timeout_sec=0.1)
            
            # 获取用户输入
            try:
                choice = input('请选择操作: ').strip()
            except EOFError:
                break
            
            if choice == '1':
                tester.send_joystick_cmd(0.2, 0.0)
            elif choice == '2':
                tester.send_joystick_cmd(-0.2, 0.0)
            elif choice == '3':
                tester.send_joystick_cmd(0.0, 0.5)
            elif choice == '4':
                tester.send_joystick_cmd(0.0, -0.5)
            elif choice == '5':
                tester.send_joystick_cmd(0.0, 0.0)
            elif choice == '6':
                tester.send_odometry_cmd(1.0, 0.0)
            elif choice == '7':
                tester.send_odometry_cmd(-0.5, 0.0)
            elif choice == '8':
                import math
                tester.send_odometry_cmd(0.0, math.pi / 2)  # 90度
            elif choice == '9':
                import math
                tester.send_odometry_cmd(0.0, -math.pi / 2)  # -90度
            elif choice == '0':
                tester.print_feedback()
            elif choice.lower() == 'q':
                print('退出测试程序')
                break
            else:
                print('无效选择，请重新输入')
    
    except KeyboardInterrupt:
        print('\n程序被中断')
    finally:
        # 发送停止命令
        tester.send_joystick_cmd(0.0, 0.0)
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()











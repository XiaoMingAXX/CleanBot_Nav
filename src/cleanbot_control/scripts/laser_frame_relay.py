#!/usr/bin/env python3
"""
激光雷达frame_id修复节点
将Gazebo的frame_id从 'cleanbot/base_footprint/laser_sensor' 改为 'laser'
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserFrameRelay(Node):
    def __init__(self):
        super().__init__('laser_frame_relay')
        
        # 订阅原始激光雷达话题
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10
        )
        
        # 发布修正后的激光雷达话题
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        self.get_logger().info('激光雷达frame_id修复节点已启动')
        self.get_logger().info('订阅: /scan_raw, 发布: /scan, frame_id修改为: laser')
    
    def scan_callback(self, msg):
        """修改frame_id并重新发布"""
        # 创建新消息并修改frame_id
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.header.frame_id = 'laser'  # 修改为正确的frame_id
        
        # 复制所有其他字段
        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = msg.ranges
        new_msg.intensities = msg.intensities
        
        # 发布修正后的消息
        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserFrameRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


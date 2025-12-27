#!/usr/bin/env python3
"""
清扫功能测试脚本
快速测试四种清扫模式
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8
import json
import time
import sys


class CleaningTester(Node):
    """清扫功能测试器"""
    
    def __init__(self):
        super().__init__('cleaning_tester')
        
        # 发布器
        self.area_cmd_pub = self.create_publisher(String, 'cleaning/area_cmd', 10)
        self.mode_cmd_pub = self.create_publisher(UInt8, 'cleaning/mode_cmd', 10)
        
        time.sleep(1.0)  # 等待发布器初始化
        
        self.get_logger().info('清扫功能测试器已启动')
    
    def set_cleaning_area(self, points):
        """设置清扫区域"""
        self.get_logger().info(f'设置清扫区域: {points}')
        
        area_data = {'points': points}
        msg = String()
        msg.data = json.dumps(area_data)
        self.area_cmd_pub.publish(msg)
        
        time.sleep(1.0)
    
    def set_mode(self, mode):
        """设置清扫模式"""
        mode_names = {
            0: '待机模式',
            1: '沿边模式',
            2: '弓形模式',
            3: '全屋模式'
        }
        
        self.get_logger().info(f'切换到 {mode_names.get(mode, "未知")}')
        
        msg = UInt8()
        msg.data = mode
        self.mode_cmd_pub.publish(msg)
        
        time.sleep(1.0)
    
    def test_edge_cleaning(self):
        """测试沿边清扫"""
        self.get_logger().info('=== 测试沿边清扫 ===')
        
        # 定义一个2x2米的方形区域
        points = [
            [0.5, 0.5],
            [2.5, 0.5],
            [2.5, 2.5],
            [0.5, 2.5]
        ]
        
        self.set_cleaning_area(points)
        self.set_mode(1)  # 沿边模式
        
        self.get_logger().info('沿边清扫已启动，请在RViz中观察')
    
    def test_boustrophedon_cleaning(self):
        """测试弓形清扫"""
        self.get_logger().info('=== 测试弓形清扫 ===')
        
        # 定义一个2x2米的方形区域
        points = [
            [0.5, 0.5],
            [2.5, 0.5],
            [2.5, 2.5],
            [0.5, 2.5]
        ]
        
        self.set_cleaning_area(points)
        self.set_mode(2)  # 弓形模式
        
        self.get_logger().info('弓形清扫已启动，请在RViz中观察')
    
    def test_auto_whole_house(self):
        """测试全屋清扫"""
        self.get_logger().info('=== 测试全屋清扫 ===')
        
        # 定义一个更大的区域
        points = [
            [-1.0, -1.0],
            [4.0, -1.0],
            [4.0, 4.0],
            [-1.0, 4.0]
        ]
        
        self.set_cleaning_area(points)
        self.set_mode(3)  # 全屋模式
        
        self.get_logger().info('全屋清扫已启动（弓形+沿边），请在RViz中观察')


def main(args=None):
    rclpy.init(args=args)
    
    tester = CleaningTester()
    
    print("\n" + "="*60)
    print("CleanBot 清扫功能测试")
    print("="*60)
    print("\n请选择测试模式：")
    print("1. 沿边清扫")
    print("2. 弓形清扫")
    print("3. 全屋清扫")
    print("0. 退出")
    print("\n提示：请确保已启动导航系统和清扫系统")
    print("="*60 + "\n")
    
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("请输入选项 (0-3): ")
    
    try:
        if choice == '1':
            tester.test_edge_cleaning()
        elif choice == '2':
            tester.test_boustrophedon_cleaning()
        elif choice == '3':
            tester.test_auto_whole_house()
        elif choice == '0':
            print("退出测试")
            tester.destroy_node()
            rclpy.shutdown()
            return
        else:
            print("无效选项")
            tester.destroy_node()
            rclpy.shutdown()
            return
        
        # 保持节点运行
        print("\n按 Ctrl+C 退出\n")
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        print("\n测试中断")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



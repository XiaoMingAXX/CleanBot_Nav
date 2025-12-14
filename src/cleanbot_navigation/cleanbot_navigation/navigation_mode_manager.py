#!/usr/bin/env python3
"""
导航模式管理节点 - 管理SLAM、导航和手动控制模式
功能：
1. 接收模式切换命令
2. 管理节点生命周期状态
3. 提供地图保存服务
4. 发布初始位姿到AMCL
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, UInt8
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import SaveMap
from std_srvs.srv import Empty
import time
import subprocess
import os


class NavigationModeManager(Node):
    """导航模式管理器"""
    
    # 模式定义
    MODE_MANUAL = 0
    MODE_MAPPING = 1
    MODE_NAVIGATION = 2
    
    MODE_NAMES = {
        MODE_MANUAL: 'manual',
        MODE_MAPPING: 'mapping',
        MODE_NAVIGATION: 'navigation'
    }
    
    def __init__(self):
        super().__init__('navigation_mode_manager')
        
        # 当前模式
        self.current_mode = self.MODE_MANUAL
        self.last_odom = None
        
        # 声明参数
        self.declare_parameter('map_save_dir', os.path.expanduser('~/cleanbot_maps'))
        self.declare_parameter('default_map_name', 'cleanbot_map')
        
        self.map_save_dir = self.get_parameter('map_save_dir').value
        self.default_map_name = self.get_parameter('default_map_name').value
        
        # 确保地图目录存在
        os.makedirs(self.map_save_dir, exist_ok=True)
        
        # 订阅器
        self.mode_cmd_sub = self.create_subscription(
            UInt8, 'navigation/mode_cmd', self.mode_cmd_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odometry/filtered', self.odom_callback, 10)
        
        # 发布器
        self.mode_status_pub = self.create_publisher(
            UInt8, 'navigation/mode_status', 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.info_pub = self.create_publisher(
            String, 'navigation/info', 10)
        
        # 服务 - 保存地图
        self.save_map_srv = self.create_service(
            Empty, 'navigation/save_map', self.save_map_callback)
        
        # 生命周期服务客户端
        self.slam_change_state_client = self.create_client(
            ChangeState, '/slam_toolbox/change_state')
        self.amcl_change_state_client = self.create_client(
            ChangeState, '/amcl/change_state')
        self.map_server_change_state_client = self.create_client(
            ChangeState, '/map_server/change_state')
        
        # 定时发布状态
        self.create_timer(0.5, self.publish_mode_status)
        
        self.get_logger().info('导航模式管理器已启动')
        self.get_logger().info(f'当前模式: {self.MODE_NAMES[self.current_mode]}')
        self.get_logger().info(f'地图保存目录: {self.map_save_dir}')
        
    def odom_callback(self, msg: Odometry):
        """里程计回调 - 保存最新位置"""
        self.last_odom = msg
    
    def mode_cmd_callback(self, msg: UInt8):
        """模式切换命令回调"""
        new_mode = msg.data
        
        if new_mode not in self.MODE_NAMES:
            self.get_logger().error(f'无效的模式: {new_mode}')
            return
        
        if new_mode == self.current_mode:
            self.get_logger().info(f'已经处于{self.MODE_NAMES[new_mode]}模式')
            return
        
        self.get_logger().info(
            f'模式切换: {self.MODE_NAMES[self.current_mode]} -> {self.MODE_NAMES[new_mode]}')
        
        # 执行模式切换
        self.switch_mode(new_mode)
    
    def switch_mode(self, new_mode: int):
        """执行模式切换"""
        old_mode = self.current_mode
        
        try:
            # 先停用旧模式的节点
            if old_mode == self.MODE_MAPPING:
                self.deactivate_slam()
            elif old_mode == self.MODE_NAVIGATION:
                self.deactivate_navigation()
            
            # 激活新模式的节点
            if new_mode == self.MODE_MAPPING:
                self.activate_slam()
            elif new_mode == self.MODE_NAVIGATION:
                self.activate_navigation()
            
            # 更新当前模式
            self.current_mode = new_mode
            
            # 发布信息
            info_msg = String()
            info_msg.data = f'mode_changed:{self.MODE_NAMES[new_mode]}'
            self.info_pub.publish(info_msg)
            
            self.get_logger().info(f'✅ 模式切换成功: {self.MODE_NAMES[new_mode]}')
            
        except Exception as e:
            self.get_logger().error(f'模式切换失败: {str(e)}')
            # 尝试恢复到手动模式
            self.current_mode = self.MODE_MANUAL
    
    def activate_slam(self):
        """激活SLAM节点"""
        self.get_logger().info('激活SLAM节点...')
        
        # 配置并激活slam_toolbox
        # slam_toolbox使用生命周期管理
        if self.slam_change_state_client.wait_for_service(timeout_sec=2.0):
            self.change_state(self.slam_change_state_client, 
                            Transition.TRANSITION_CONFIGURE)
            time.sleep(0.5)
            self.change_state(self.slam_change_state_client, 
                            Transition.TRANSITION_ACTIVATE)
            self.get_logger().info('✅ SLAM节点已激活')
        else:
            self.get_logger().warn('⚠️ SLAM节点服务未找到')
    
    def deactivate_slam(self):
        """停用SLAM节点"""
        self.get_logger().info('停用SLAM节点...')
        
        if self.slam_change_state_client.wait_for_service(timeout_sec=2.0):
            self.change_state(self.slam_change_state_client, 
                            Transition.TRANSITION_DEACTIVATE)
            time.sleep(0.3)
            self.change_state(self.slam_change_state_client, 
                            Transition.TRANSITION_CLEANUP)
            self.get_logger().info('✅ SLAM节点已停用')
        else:
            self.get_logger().warn('⚠️ SLAM节点服务未找到')
    
    def activate_navigation(self):
        """激活导航节点（AMCL + Map Server + Nav2）"""
        self.get_logger().info('激活导航节点...')
        
        # 1. 激活Map Server
        if self.map_server_change_state_client.wait_for_service(timeout_sec=2.0):
            self.change_state(self.map_server_change_state_client, 
                            Transition.TRANSITION_CONFIGURE)
            time.sleep(0.3)
            self.change_state(self.map_server_change_state_client, 
                            Transition.TRANSITION_ACTIVATE)
            self.get_logger().info('✅ Map Server已激活')
        else:
            self.get_logger().warn('⚠️ Map Server服务未找到')
        
        time.sleep(0.5)
        
        # 2. 激活AMCL
        if self.amcl_change_state_client.wait_for_service(timeout_sec=2.0):
            self.change_state(self.amcl_change_state_client, 
                            Transition.TRANSITION_CONFIGURE)
            time.sleep(0.3)
            self.change_state(self.amcl_change_state_client, 
                            Transition.TRANSITION_ACTIVATE)
            self.get_logger().info('✅ AMCL已激活')
        else:
            self.get_logger().warn('⚠️ AMCL服务未找到')
        
        time.sleep(0.5)
        
        # 3. 发布初始位姿（使用当前里程计位置）
        self.publish_initial_pose()
        
        self.get_logger().info('✅ 导航节点已激活')
    
    def deactivate_navigation(self):
        """停用导航节点"""
        self.get_logger().info('停用导航节点...')
        
        # 停用AMCL
        if self.amcl_change_state_client.wait_for_service(timeout_sec=2.0):
            self.change_state(self.amcl_change_state_client, 
                            Transition.TRANSITION_DEACTIVATE)
            time.sleep(0.2)
            self.change_state(self.amcl_change_state_client, 
                            Transition.TRANSITION_CLEANUP)
        
        # 停用Map Server
        if self.map_server_change_state_client.wait_for_service(timeout_sec=2.0):
            self.change_state(self.map_server_change_state_client, 
                            Transition.TRANSITION_DEACTIVATE)
            time.sleep(0.2)
            self.change_state(self.map_server_change_state_client, 
                            Transition.TRANSITION_CLEANUP)
        
        self.get_logger().info('✅ 导航节点已停用')
    
    def change_state(self, client, transition_id):
        """改变节点生命周期状态"""
        request = ChangeState.Request()
        request.transition.id = transition_id
        
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                if future.result().success:
                    return True
                else:
                    self.get_logger().error(f'状态切换失败: {client.srv_name}')
                    return False
            else:
                self.get_logger().error(f'状态切换超时: {client.srv_name}')
                return False
        except Exception as e:
            self.get_logger().error(f'状态切换异常: {str(e)}')
            return False
    
    def publish_initial_pose(self):
        """发布初始位姿到AMCL"""
        if self.last_odom is None:
            self.get_logger().warn('⚠️ 没有里程计数据，使用原点作为初始位姿')
            
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        if self.last_odom is not None:
            # 使用当前里程计位置作为初始位姿
            pose_msg.pose.pose = self.last_odom.pose.pose
            self.get_logger().info(
                f'发布初始位姿: x={self.last_odom.pose.pose.position.x:.2f}, '
                f'y={self.last_odom.pose.pose.position.y:.2f}')
        else:
            # 使用原点
            pose_msg.pose.pose.position.x = 0.0
            pose_msg.pose.pose.position.y = 0.0
            pose_msg.pose.pose.position.z = 0.0
            pose_msg.pose.pose.orientation.w = 1.0
            self.get_logger().info('发布初始位姿: 原点(0, 0)')
        
        # 设置协方差（初始不确定性）
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06854
        ]
        
        # 发布多次确保接收
        for _ in range(3):
            self.initialpose_pub.publish(pose_msg)
            time.sleep(0.1)
    
    def save_map_callback(self, request, response):
        """保存地图服务回调"""
        self.get_logger().info('收到保存地图请求')
        
        if self.current_mode != self.MODE_MAPPING:
            self.get_logger().warn('⚠️ 当前不在建图模式，无法保存地图')
            return response
        
        try:
            # 生成地图文件名（带时间戳）
            import datetime
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            map_name = f'{self.default_map_name}_{timestamp}'
            map_path = os.path.join(self.map_save_dir, map_name)
            
            # 调用map_saver服务保存地图
            self.get_logger().info(f'保存地图到: {map_path}')
            
            # 使用ros2 run命令保存地图
            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path,
                '--ros-args', '-p', 'save_map_timeout:=5000'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info(f'✅ 地图保存成功: {map_path}')
                
                # 发布信息
                info_msg = String()
                info_msg.data = f'map_saved:{map_name}'
                self.info_pub.publish(info_msg)
            else:
                self.get_logger().error(f'地图保存失败: {result.stderr}')
                
        except Exception as e:
            self.get_logger().error(f'保存地图异常: {str(e)}')
        
        return response
    
    def publish_mode_status(self):
        """定期发布当前模式状态"""
        msg = UInt8()
        msg.data = self.current_mode
        self.mode_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationModeManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

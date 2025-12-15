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
from nav2_msgs.srv import SaveMap, LoadMap
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
        self.map_path_sub = self.create_subscription(
            String, 'navigation/map_path', self.map_path_callback, 10)
        
        # 发布器
        self.mode_status_pub = self.create_publisher(
            UInt8, 'navigation/mode_status', 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.info_pub = self.create_publisher(
            String, 'navigation/info', 10)
        
        # 服务
        self.save_map_srv = self.create_service(
            Empty, 'navigation/save_map', self.save_map_callback)
        
        # 选中的地图路径
        self.current_map_path = None
        
        # 生命周期服务客户端
        self.slam_change_state_client = self.create_client(
            ChangeState, '/slam_toolbox/change_state')
        self.amcl_change_state_client = self.create_client(
            ChangeState, '/amcl/change_state')
        self.map_server_change_state_client = self.create_client(
            ChangeState, '/map_server/change_state')
        
        # Nav2核心节点lifecycle客户端
        self.controller_change_state_client = self.create_client(
            ChangeState, '/controller_server/change_state')
        self.planner_change_state_client = self.create_client(
            ChangeState, '/planner_server/change_state')
        self.bt_navigator_change_state_client = self.create_client(
            ChangeState, '/bt_navigator/change_state')
        self.behavior_change_state_client = self.create_client(
            ChangeState, '/behavior_server/change_state')
        
        # Map Server的LoadMap服务客户端
        self.load_map_client = self.create_client(
            LoadMap, '/map_server/load_map')
        
        # 定时发布状态
        self.create_timer(0.5, self.publish_mode_status)
        
        self.get_logger().info('导航模式管理器已启动')
        self.get_logger().info(f'当前模式: {self.MODE_NAMES[self.current_mode]}')
        self.get_logger().info(f'地图保存目录: {self.map_save_dir}')
        
    def odom_callback(self, msg: Odometry):
        """里程计回调 - 保存最新位置"""
        self.last_odom = msg
    
    def map_path_callback(self, msg: String):
        """地图路径回调 - 设置要加载的地图路径"""
        self.current_map_path = msg.data
        self.get_logger().info(f'收到地图路径: {self.current_map_path}')
    
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
                self.get_logger().info('停用SLAM节点...')
                self.deactivate_slam()
            elif old_mode == self.MODE_NAVIGATION:
                self.get_logger().info('停用导航节点...')
                self.deactivate_navigation()
            
            # 激活新模式的节点
            success = True
            if new_mode == self.MODE_MAPPING:
                success = self.activate_slam()
            elif new_mode == self.MODE_NAVIGATION:
                success = self.activate_navigation()
            
            if not success:
                self.get_logger().error(f'❌ 模式切换失败: {self.MODE_NAMES[new_mode]}')
                # 恢复到手动模式
                self.current_mode = self.MODE_MANUAL
                # activate函数已经发送了详细的mode_failed消息，这里不再重复发送
                return
            
            # 更新当前模式
            self.current_mode = new_mode
            
            # 等待一小段时间确保节点完全启动
            time.sleep(0.5)
            
            # 发布成功信息（真实状态变化后）
            info_msg = String()
            info_msg.data = f'mode_changed:{self.MODE_NAMES[new_mode]}'
            self.info_pub.publish(info_msg)
            
            self.get_logger().info(f'✅ 模式切换成功: {self.MODE_NAMES[new_mode]}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 模式切换异常: {str(e)}')
            import traceback
            traceback.print_exc()
            # 尝试恢复到手动模式
            self.current_mode = self.MODE_MANUAL
            info_msg = String()
            info_msg.data = f'mode_failed:系统异常:{str(e)[:30]}'
            self.info_pub.publish(info_msg)
    
    def activate_slam(self):
        """激活SLAM节点 - 使用异步方式避免超时"""
        self.get_logger().info('激活SLAM节点...')
        
        # 发送状态信息
        info_msg = String()
        info_msg.data = 'slam_activating'
        self.info_pub.publish(info_msg)
        
        if not self.slam_change_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('❌ SLAM节点服务未找到')
            info_msg = String()
            info_msg.data = 'mode_failed:SLAM节点服务未找到'
            self.info_pub.publish(info_msg)
            return False
        
        # 异步发送configure（不等待完成）
        self.get_logger().info('发送configure请求到SLAM...')
        req_cfg = ChangeState.Request()
        req_cfg.transition.id = Transition.TRANSITION_CONFIGURE
        self.slam_change_state_client.call_async(req_cfg)
        time.sleep(1.0)
        
        # 异步发送activate（不等待完成）
        self.get_logger().info('发送activate请求到SLAM...')
        req_act = ChangeState.Request()
        req_act.transition.id = Transition.TRANSITION_ACTIVATE
        self.slam_change_state_client.call_async(req_act)
        time.sleep(2.0)  # 等待SLAM启动
        
        self.get_logger().info('✅ SLAM启动命令已发送')
        return True
    
    def deactivate_slam(self):
        """停用SLAM节点"""
        self.get_logger().info('停用SLAM节点...')
        
        if self.slam_change_state_client.wait_for_service(timeout_sec=2.0):
            # 异步停用，不等待完成
            req_deact = ChangeState.Request()
            req_deact.transition.id = Transition.TRANSITION_DEACTIVATE
            self.slam_change_state_client.call_async(req_deact)
            time.sleep(0.5)
            
            req_cleanup = ChangeState.Request()
            req_cleanup.transition.id = Transition.TRANSITION_CLEANUP
            self.slam_change_state_client.call_async(req_cleanup)
            time.sleep(0.5)
            
            self.get_logger().info('✅ SLAM停用命令已发送')
        else:
            self.get_logger().warn('⚠️ SLAM节点服务未找到')
    
    def activate_navigation(self):
        """激活导航节点（Map Server + AMCL）"""
        self.get_logger().info('激活导航节点...')
        
        # 发送状态信息
        info_msg = String()
        info_msg.data = 'nav_activating'
        self.info_pub.publish(info_msg)
        
        # 检查是否选择了地图
        if self.current_map_path is None:
            self.get_logger().error('❌ 未选择地图')
            info_msg = String()
            info_msg.data = 'mode_failed:请先选择地图'
            self.info_pub.publish(info_msg)
            return False
        
        # 1. 启动Map Server并加载地图
        self.get_logger().info(f'启动Map Server，加载地图: {self.current_map_path}')
        if not self.map_server_change_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('❌ Map Server服务未找到')
            info_msg = String()
            info_msg.data = 'mode_failed:Map Server服务未找到'
            self.info_pub.publish(info_msg)
            return False
        
        # 异步发送configure
        req_cfg = ChangeState.Request()
        req_cfg.transition.id = Transition.TRANSITION_CONFIGURE
        self.map_server_change_state_client.call_async(req_cfg)
        time.sleep(1.0)
        
        # 异步发送activate
        req_act = ChangeState.Request()
        req_act.transition.id = Transition.TRANSITION_ACTIVATE
        self.map_server_change_state_client.call_async(req_act)
        time.sleep(1.0)
        
        # 加载地图
        if not self.load_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('❌ LoadMap服务未找到')
            info_msg = String()
            info_msg.data = 'mode_failed:LoadMap服务未找到'
            self.info_pub.publish(info_msg)
            return False
        
        load_req = LoadMap.Request()
        load_req.map_url = self.current_map_path
        self.load_map_client.call_async(load_req)  # 异步调用，不等待
        
        self.get_logger().info('✅ 地图加载命令已发送')
        time.sleep(1.0)  # 短暂等待地图加载
        
        # 2. 启动AMCL
        if not self.amcl_change_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('❌ AMCL服务未找到')
            info_msg = String()
            info_msg.data = 'mode_failed:AMCL服务未找到'
            self.info_pub.publish(info_msg)
            return False
        
        # 异步配置AMCL（不等待完成）
        self.get_logger().info('发送configure请求到AMCL...')
        req_cfg = ChangeState.Request()
        req_cfg.transition.id = Transition.TRANSITION_CONFIGURE
        self.amcl_change_state_client.call_async(req_cfg)
        time.sleep(1.0)
        
        # 异步激活AMCL（不等待完成）
        self.get_logger().info('发送activate请求到AMCL...')
        req_act = ChangeState.Request()
        req_act.transition.id = Transition.TRANSITION_ACTIVATE
        self.amcl_change_state_client.call_async(req_act)
        time.sleep(2.0)  # 等待AMCL初始化
        
        self.get_logger().info('✅ AMCL启动命令已发送')
        
        # 发布初始位姿
        time.sleep(0.3)
        self.publish_initial_pose()
        
        # 3. 启动Nav2核心节点
        self.get_logger().info('启动Nav2导航栈...')
        nav2_nodes = [
            (self.controller_change_state_client, 'controller_server'),
            (self.planner_change_state_client, 'planner_server'),
            (self.bt_navigator_change_state_client, 'bt_navigator'),
            (self.behavior_change_state_client, 'behavior_server')
        ]
        
        for client, name in nav2_nodes:
            if client.wait_for_service(timeout_sec=1.0):
                # 异步configure
                req_cfg = ChangeState.Request()
                req_cfg.transition.id = Transition.TRANSITION_CONFIGURE
                client.call_async(req_cfg)
                time.sleep(0.3)
                
                # 异步activate
                req_act = ChangeState.Request()
                req_act.transition.id = Transition.TRANSITION_ACTIVATE
                client.call_async(req_act)
                time.sleep(0.3)
                
                self.get_logger().info(f'  ✅ {name}启动命令已发送')
            else:
                self.get_logger().warn(f'  ⚠️ {name}服务未找到')
        
        self.get_logger().info('✅ 完整导航栈已启动')
        return True
    
    def deactivate_navigation(self):
        """停用导航节点（Nav2 + AMCL + Map Server）"""
        self.get_logger().info('停用导航节点...')
        
        # 1. 停用Nav2核心节点
        nav2_nodes = [
            (self.behavior_change_state_client, 'behavior_server'),
            (self.bt_navigator_change_state_client, 'bt_navigator'),
            (self.planner_change_state_client, 'planner_server'),
            (self.controller_change_state_client, 'controller_server')
        ]
        
        for client, name in nav2_nodes:
            if client.wait_for_service(timeout_sec=1.0):
                # 异步deactivate
                req_deact = ChangeState.Request()
                req_deact.transition.id = Transition.TRANSITION_DEACTIVATE
                client.call_async(req_deact)
                time.sleep(0.2)
                
                # 异步cleanup
                req_cleanup = ChangeState.Request()
                req_cleanup.transition.id = Transition.TRANSITION_CLEANUP
                client.call_async(req_cleanup)
                time.sleep(0.2)
        
        # 2. 停用AMCL
        if self.amcl_change_state_client.wait_for_service(timeout_sec=1.0):
            req_deact = ChangeState.Request()
            req_deact.transition.id = Transition.TRANSITION_DEACTIVATE
            self.amcl_change_state_client.call_async(req_deact)
            time.sleep(0.2)
            
            req_cleanup = ChangeState.Request()
            req_cleanup.transition.id = Transition.TRANSITION_CLEANUP
            self.amcl_change_state_client.call_async(req_cleanup)
            time.sleep(0.2)
        
        # 3. 停用Map Server
        if self.map_server_change_state_client.wait_for_service(timeout_sec=1.0):
            req_deact = ChangeState.Request()
            req_deact.transition.id = Transition.TRANSITION_DEACTIVATE
            self.map_server_change_state_client.call_async(req_deact)
            time.sleep(0.2)
            
            req_cleanup = ChangeState.Request()
            req_cleanup.transition.id = Transition.TRANSITION_CLEANUP
            self.map_server_change_state_client.call_async(req_cleanup)
            time.sleep(0.2)
        
        self.get_logger().info('✅ 导航节点已停用')
    
    def change_state(self, client, transition_id):
        """改变节点生命周期状态"""
        request = ChangeState.Request()
        request.transition.id = transition_id
        
        # 根据转换类型设置不同的超时时间
        # CONFIGURE需要更长时间（SLAM节点加载地图、初始化等）
        if transition_id == Transition.TRANSITION_CONFIGURE:
            timeout = 15.0  # 配置阶段：15秒
        else:
            timeout = 10.0  # 其他阶段：10秒
        
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f'✅ 状态切换成功: {client.srv_name}')
                    return True
                else:
                    self.get_logger().error(f'❌ 状态切换失败: {client.srv_name}')
                    return False
            else:
                self.get_logger().error(f'❌ 状态切换超时({timeout}秒): {client.srv_name}')
                return False
        except Exception as e:
            self.get_logger().error(f'❌ 状态切换异常: {str(e)}')
            return False
    
    def get_state(self, change_state_client):
        """获取lifecycle节点当前状态
        返回值: 1=unconfigured, 2=inactive, 3=active, 4=finalized
        """
        # 从change_state客户端的服务名获取节点名
        # /node_name/change_state -> /node_name/get_state
        service_name = change_state_client.srv_name.replace('change_state', 'get_state')
        
        # 创建临时的get_state客户端
        get_state_client = self.create_client(GetState, service_name)
        
        if not get_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'GetState服务未找到: {service_name}')
            return 0  # 未知状态
        
        try:
            request = GetState.Request()
            future = get_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                state_id = future.result().current_state.id
                self.get_logger().info(f'查询到状态: {state_id} ({service_name})')
                return state_id
            else:
                self.get_logger().warn(f'获取状态超时: {service_name}')
                return 0
        except Exception as e:
            self.get_logger().error(f'获取状态异常: {str(e)}')
            return 0
        finally:
            # 销毁临时客户端
            self.destroy_client(get_state_client)
    
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
            # 发布失败信息
            info_msg = String()
            info_msg.data = 'map_save_failed:不在建图模式'
            self.info_pub.publish(info_msg)
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
                '--ros-args', '-p', 'save_map_timeout:=5000.0'  # 必须是浮点数
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info(f'✅ 地图保存成功: {map_path}')
                
                # 发布成功信息
                info_msg = String()
                info_msg.data = f'map_saved:{map_name}'
                self.info_pub.publish(info_msg)
            else:
                self.get_logger().error(f'地图保存失败: {result.stderr}')
                
                # 发布失败信息
                info_msg = String()
                info_msg.data = f'map_save_failed:{result.stderr[:50]}'
                self.info_pub.publish(info_msg)
                
        except Exception as e:
            self.get_logger().error(f'保存地图异常: {str(e)}')
            
            # 发布异常信息
            info_msg = String()
            info_msg.data = f'map_save_failed:{str(e)[:50]}'
            self.info_pub.publish(info_msg)
        
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

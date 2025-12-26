#!/usr/bin/env python3
"""
清扫任务管理节点 - 重构版
负责生成清扫路径并管理目标点序列导航
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import UInt8, String, Float32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
import cv2
import math
import time
import os
from typing import List, Tuple, Optional
from enum import IntEnum


class CleaningMode(IntEnum):
    """清扫模式枚举"""
    STANDBY = 0
    EDGE = 1
    BOUSTROPHEDON = 2
    AUTO = 3


class CleaningTaskNode(Node):
    """清扫任务管理节点 - 使用目标点序列+GridBased规划器"""
    
    def __init__(self):
        super().__init__('cleaning_task_node')
        
        # 声明参数
        self.declare_parameter('waypoint_spacing', 0.5)  # 路点间距(米)
        self.declare_parameter('robot_radius', 0.15)  # 机器人半径(米)
        self.declare_parameter('edge_offset', 0.35)  # 沿边偏移距离(米)
        self.declare_parameter('coverage_stripe_width', 0.3)  # 弓形覆盖条带宽度(米)
        self.declare_parameter('corner_radius', 0.3)  # 转角圆弧半径(米)
        
        # 获取参数
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.edge_offset = self.get_parameter('edge_offset').value
        self.coverage_stripe_width = self.get_parameter('coverage_stripe_width').value
        self.corner_radius = self.get_parameter('corner_radius').value
        
        # 状态变量
        self.current_mode = CleaningMode.STANDBY
        self.current_pose: Optional[PoseStamped] = None
        self.static_map: Optional[OccupancyGrid] = None
        self.waypoints: List[PoseStamped] = []  # 当前清扫路径的所有航点
        self.current_waypoint_idx = 0  # 当前正在导航到第几个航点
        self.current_goal_handle = None  # 当前活跃的目标句柄
        self.is_cleaning = False
        self.pending_cleaning_mode: Optional[CleaningMode] = None  # 待执行的清扫模式
        
        # 边界数据（用于可视化）
        self.outer_boundary: List[Tuple[float, float]] = []  # 清扫区域边界
        self.obstacles: List[List[Tuple[float, float]]] = []  # 障碍物边界列表
        self.cleaning_area_mask: Optional[np.ndarray] = None  # 清扫区域mask（用于安全检查）
        
        # 订阅当前位置（从AMCL）
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)
        
        # 订阅清扫模式命令
        self.mode_sub = self.create_subscription(
            UInt8, 'cleaning/mode_cmd', self.mode_callback, 10)
        
        # 地图服务客户端
        self.map_client = self.create_client(GetMap, '/map_server/map')
        
        # NavigateToPose action客户端
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 发布器
        self.task_info_pub = self.create_publisher(String, 'cleaning/task_info', 10)
        self.progress_pub = self.create_publisher(Float32, 'cleaning/progress', 10)
        self.path_pub = self.create_publisher(Path, 'cleaning/planned_path', 10)
        self.waypoint_markers_pub = self.create_publisher(MarkerArray, 'cleaning/waypoint_markers', 10)
        self.boundary_markers_pub = self.create_publisher(MarkerArray, 'cleaning/boundary_markers', 10)
        
        # 发布规划器选择（切换到GridBased）
        selector_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.planner_selector_pub = self.create_publisher(
            String, 'planner_selector', selector_qos)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('清扫任务管理节点已启动 (重构版)')
        self.get_logger().info(f'参数: 路点间距={self.waypoint_spacing}m')
        self.get_logger().info(f'      机器人半径={self.robot_radius}m, 沿边偏移={self.edge_offset}m')
        self.get_logger().info('使用GridBased规划器 + 单目标点顺序导航')
        self.get_logger().info('=' * 60)
        
        # 确保使用GridBased规划器
        self._publish_planner_selection('GridBased')
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """位置回调"""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.current_pose = pose_stamped
        
        if not hasattr(self, '_first_pose_received'):
            self._first_pose_received = True
            self.get_logger().info(
                f'✅ 收到机器人位置: ({pose_stamped.pose.position.x:.2f}, '
                f'{pose_stamped.pose.position.y:.2f})')
    
    def mode_callback(self, msg: UInt8):
        """模式切换回调"""
        new_mode = CleaningMode(msg.data)
        
        if new_mode == self.current_mode:
            return
        
        self.current_mode = new_mode
        mode_names = {
            CleaningMode.STANDBY: '待机',
            CleaningMode.EDGE: '沿边',
            CleaningMode.BOUSTROPHEDON: '弓形',
            CleaningMode.AUTO: '自动全屋'
        }
        
        self.get_logger().info(f'📢 收到清扫模式切换: {mode_names[new_mode]}')
        
        if new_mode == CleaningMode.STANDBY:
            self.stop_cleaning()
        else:
            self.start_cleaning_task(new_mode)
    
    def stop_cleaning(self):
        """停止清扫任务"""
        if self.is_cleaning:
            self.get_logger().info('⏹ 停止清扫任务')
            # 取消当前活跃的目标
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            self.is_cleaning = False
            self.waypoints.clear()
            self.current_waypoint_idx = 0
            self._publish_task_info('stopped:清扫已停止')
    
    def start_cleaning_task(self, mode: CleaningMode):
        """启动清扫任务"""
        if self.current_pose is None:
            self.get_logger().error('❌ 未收到机器人位置，无法启动清扫')
            self._publish_task_info('error:未收到机器人位置')
            return
        
        mode_names = {
            CleaningMode.EDGE: '沿边',
            CleaningMode.BOUSTROPHEDON: '弓形',
            CleaningMode.AUTO: '自动全屋'
        }
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'🚀 启动{mode_names[mode]}清扫任务')
        self.get_logger().info(f'起始位置: ({self.current_pose.pose.position.x:.2f}, '
                             f'{self.current_pose.pose.position.y:.2f})')
        
        # 保存模式用于异步回调
        self.pending_cleaning_mode = mode
        
        # 1. 异步获取静态地图
        self._fetch_static_map_async()
    
    def _fetch_static_map_async(self):
        """异步获取静态地图"""
        self.get_logger().info('等待地图服务 /map_server/map ...')
        
        if not self.map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('地图服务不可用')
            self._publish_task_info('error:地图服务不可用')
            return
        
        self.get_logger().info('发送地图请求...')
        request = GetMap.Request()
        future = self.map_client.call_async(request)
        future.add_done_callback(self._map_response_callback)
    
    def _map_response_callback(self, future):
        """地图响应回调"""
        try:
            response = future.result()
            if response is None:
                self.get_logger().error('地图服务响应为空')
                self._publish_task_info('error:地图响应为空')
                return
            
            self.static_map = response.map
            self.get_logger().info(
                f'✅ 获取静态地图: {self.static_map.info.width}x{self.static_map.info.height}, '
                f'分辨率={self.static_map.info.resolution:.3f}m')
            
            # 继续执行清扫任务
            self._continue_cleaning_task()
            
        except Exception as e:
            self.get_logger().error(f'获取地图失败: {str(e)}')
            self._publish_task_info(f'error:获取地图失败-{str(e)}')
    
    def _continue_cleaning_task(self):
        """继续执行清扫任务（获取地图后）"""
        mode = self.pending_cleaning_mode
        mode_names = {
            CleaningMode.EDGE: '沿边',
            CleaningMode.BOUSTROPHEDON: '弓形',
            CleaningMode.AUTO: '自动全屋'
        }
        
        # 2. 根据模式生成航点
        self.get_logger().info(f'[1/3] 生成{mode_names[mode]}清扫路径...')
        start_time = time.time()
        
        if mode == CleaningMode.EDGE:
            self.waypoints = self._generate_edge_path()
        elif mode == CleaningMode.BOUSTROPHEDON:
            self.waypoints = self._generate_boustrophedon_path()
        elif mode == CleaningMode.AUTO:
            self.waypoints = self._generate_auto_coverage_path()
        
        if not self.waypoints:
            self.get_logger().error('❌ 路径生成失败')
            self._publish_task_info('error:路径生成失败')
            return
        
        elapsed = time.time() - start_time
        self.get_logger().info(f'✅ 路径生成完成: {len(self.waypoints)}个航点 (耗时{elapsed:.2f}秒)')
        
        # 3. 发布完整路径（供前端显示）
        self._publish_path()
        
        # 4. 开始执行导航
        self.is_cleaning = True
        self.current_waypoint_idx = 0
        self.current_goal_handle = None
        
        self.get_logger().info('[2/3] 开始执行导航...')
        self._publish_task_info(f'executing:{mode_names[mode]}清扫中')
        self._send_next_waypoint()
    
    def _generate_edge_path(self) -> List[PoseStamped]:
        """生成沿边清扫路径（严格按照edge_planner.cpp逻辑）"""
        self.get_logger().info('========== 沿边清扫路径规划 ==========')
        self.get_logger().info('[1/4] 提取清扫区域边界（向内偏移）...')
        
        # 提取清扫区域边界（已经向内偏移35cm，与墙体保持安全距离）
        cleaning_boundary = self._extract_inset_boundary()
        if len(cleaning_boundary) < 3:
            self.get_logger().error('  清扫区域边界提取失败')
            return []
        
        self.get_logger().info(f'✓ 清扫区域边界: {len(cleaning_boundary)}个顶点, 偏移={self.edge_offset:.2f}m')
        
        # 找最近边界点
        self.get_logger().info('[2/4] 寻找最近清扫边界点...')
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        
        closest_idx = 0
        min_dist = float('inf')
        for i, (bx, by) in enumerate(cleaning_boundary):
            dist = math.sqrt((bx - start_x)**2 + (by - start_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        self.get_logger().info(f'✓ 最近边界点索引={closest_idx}, 距离={min_dist:.2f}m')
        
        # 生成路径
        waypoints = []
        
        # 从起点插值到最近边界点（固定0.3m间距）
        self.get_logger().info('[3/4] 生成起点到清扫边界的路径...')
        dx = cleaning_boundary[closest_idx][0] - start_x
        dy = cleaning_boundary[closest_idx][1] - start_y
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist > 0.3:
            num_steps = max(1, int(dist / 0.3))
            for j in range(num_steps + 1):
                t = j / num_steps
                x = start_x + t * dx
                y = start_y + t * dy
                # 沿边清扫使用宽松的安全检查
                if self._is_position_safe(x, y, strict=False):
                    yaw = math.atan2(dy, dx)
                    waypoints.append(self._create_pose_simple(x, y, yaw))
        else:
            # 距离太近，直接添加起点
            if self._is_position_safe(start_x, start_y, strict=False):
                waypoints.append(self._create_pose_simple(start_x, start_y, 0.0))
        
        # 沿清扫区域边界一圈
        self.get_logger().info('[4/4] 沿清扫区域边界生成完整路径...')
        
        skipped_count = 0
        # 遍历所有边界点，形成闭环
        for count in range(len(cleaning_boundary)):
            idx = (closest_idx + count) % len(cleaning_boundary)
            next_idx = (closest_idx + count + 1) % len(cleaning_boundary)
            
            # 当前边界点坐标
            curr_x, curr_y = cleaning_boundary[idx]
            next_x, next_y = cleaning_boundary[next_idx]
            
            # 计算朝向
            dx = next_x - curr_x
            dy = next_y - curr_y
            dist = math.sqrt(dx*dx + dy*dy)
            yaw = math.atan2(dy, dx)
            
            # 添加当前边界点（沿边清扫使用宽松检查）
            if self._is_position_safe(curr_x, curr_y, strict=False):
                waypoints.append(self._create_pose_simple(curr_x, curr_y, yaw))
            else:
                skipped_count += 1
            
            # 边界点之间插值（固定0.2m间距）
            if dist > 0.2:
                num_steps = int(dist / 0.2)
                
                for j in range(1, num_steps):
                    t = j / num_steps
                    x = curr_x + t * dx
                    y = curr_y + t * dy
                    # 沿边清扫使用宽松检查
                    if self._is_position_safe(x, y, strict=False):
                        waypoints.append(self._create_pose_simple(x, y, yaw))
                    else:
                        skipped_count += 1
        
        if skipped_count > 0:
            self.get_logger().info(f'⚠ 跳过了{skipped_count}个不安全点（障碍物附近或地图外）')
        
        self.get_logger().info(f'✓ 沿边路径生成完成: {len(waypoints)}个路点')
        return waypoints
    
    def _generate_boustrophedon_path(self) -> List[PoseStamped]:
        """生成弓形清扫路径（严格按照boustrophedon_planner.cpp逻辑）"""
        self.get_logger().info('========== 弓形清扫路径规划 ==========')
        self.get_logger().info('[1/3] 提取清扫区域边界（向内偏移）...')
        
        # 提取清扫区域边界（向内偏移，与墙体保持安全距离）
        cleaning_boundary = self._extract_inset_boundary()
        if len(cleaning_boundary) < 3:
            self.get_logger().error('  清扫区域边界提取失败')
            return []
        
        # 计算包围盒
        xs = [p[0] for p in cleaning_boundary]
        ys = [p[1] for p in cleaning_boundary]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        self.get_logger().info(f'✓ 清扫区域边界: {len(cleaning_boundary)}个顶点, 偏移={self.edge_offset:.2f}m')
        self.get_logger().info(f'  包围盒: x[{min_x:.2f}, {max_x:.2f}], y[{min_y:.2f}, {max_y:.2f}]')
        
        # 弓字形参数（按照C++逻辑）
        stripe_width = self.robot_radius * 2.0  # 机器人直径
        sample_spacing = 0.25  # 固定0.25m采样间距
        
        self.get_logger().info(f'[2/3] 生成弓形路径...')
        self.get_logger().info(f'  条带宽度={stripe_width:.3f}m, 采样间距={sample_spacing:.3f}m')
        
        # 生成弓形路径
        waypoints = []
        y = min_y
        direction = 1  # 1: 左到右, -1: 右到左
        line_count = 0
        
        while y <= max_y:
            line_points = []
            
            if direction == 1:
                # 左到右扫描
                x = min_x
                while x <= max_x:
                    # 检查是否在清扫区域内且安全（避开障碍物）
                    if self._is_point_in_polygon(x, y, cleaning_boundary) and self._is_position_safe(x, y):
                        line_points.append((x, y))
                    x += sample_spacing
            else:
                # 右到左扫描
                x = max_x
                while x >= min_x:
                    if self._is_point_in_polygon(x, y, cleaning_boundary) and self._is_position_safe(x, y):
                        line_points.append((x, y))
                    x -= sample_spacing
            
            # 添加这条线到路径
            for i, (px, py) in enumerate(line_points):
                # 计算朝向
                if i < len(line_points) - 1:
                    yaw = math.atan2(
                        line_points[i+1][1] - py,
                        line_points[i+1][0] - px)
                elif i > 0:
                    yaw = math.atan2(
                        py - line_points[i-1][1],
                        px - line_points[i-1][0])
                else:
                    yaw = 0.0
                
                waypoints.append(self._create_pose_simple(px, py, yaw))
            
            line_count += 1
            y += stripe_width
            direction *= -1
        
        self.get_logger().info(f'[3/3] 弓形路径规划完成: {line_count}条扫描线, {len(waypoints)}个路点')
        return waypoints
    
    def _generate_auto_coverage_path(self) -> List[PoseStamped]:
        """
        生成自动全屋清扫路径（完整版：Cell Decomposition + ACO求解TSP）
        算法流程：
        1. 对清扫区域分块（Cell Decomposition）
        2. 为每个子块生成弓形覆盖路径
        3. 构建节点图和cost矩阵
        4. 使用蚁群算法求解TSP获得最优访问顺序
        5. 恢复完整路径
        """
        self.get_logger().info('========== 自动全屋覆盖路径规划 ==========')
        
        # 步骤1: 地图预处理，提取清扫区域边界和障碍物
        self.get_logger().info('[1/5] 地图预处理: 提取清扫区域边界和障碍物...')
        cleaning_boundary, obstacles = self._preprocess_map()
        
        if len(cleaning_boundary) < 3:
            self.get_logger().error('地图预处理失败: 未找到有效的清扫区域')
            return []
        
        self.get_logger().info(f'✓ 清扫区域边界: {len(cleaning_boundary)}个顶点, 偏移={self.edge_offset:.2f}m')
        self.get_logger().info(f'✓ 障碍物: {len(obstacles)}个')
        
        # 步骤2: Cell Decomposition - 清扫区域分块
        self.get_logger().info('[2/5] 执行Cell Decomposition分块算法...')
        cells = self._cell_decomposition(cleaning_boundary, obstacles)
        
        if not cells:
            self.get_logger().error('Cell Decomposition失败: 未生成有效子区域')
            return []
        
        self.get_logger().info(f'✓ 分解完成: 共{len(cells)}个子区域')
        
        # 步骤3: 为每个Cell生成覆盖路径
        self.get_logger().info('[3/5] 为每个子区域生成弓形覆盖路径...')
        for i, cell in enumerate(cells):
            self._generate_cell_coverage_path(cell)
            self.get_logger().debug(f'  子区域[{i}]: {len(cell["coverage_path"])}个航点')
        self.get_logger().info('✓ 所有子区域路径生成完成')
        
        # 步骤4: 使用蚁群算法求解TSP，优化访问顺序
        self.get_logger().info('[4/5] 使用蚁群算法求解TSP，优化子区域访问顺序...')
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        start_pos = (start_x, start_y)
        visit_order = self._solve_tsp_with_aco(cells, start_pos)
        
        order_str = ' '.join(str(idx) for idx in visit_order)
        self.get_logger().info(f'✓ TSP求解完成, 最优访问顺序: [{order_str}]')
        
        # 步骤5: 按照最优顺序连接所有子路径
        self.get_logger().info('[5/5] 连接所有子路径生成完整覆盖路径...')
        waypoints = []
        
        # 从起点到第一个Cell的入口
        if visit_order:
            connection = self._connect_points(start_pos, cells[visit_order[0]]['entry'])
            waypoints.extend(connection)
        
        # 遍历所有Cell
        for i, cell_idx in enumerate(visit_order):
            cell = cells[cell_idx]
            
            # 添加Cell内部的覆盖路径
            waypoints.extend(cell['coverage_path'])
            
            # 如果不是最后一个Cell，连接到下一个Cell
            if i < len(visit_order) - 1:
                next_cell_idx = visit_order[i + 1]
                connection = self._connect_points(cell['exit'], cells[next_cell_idx]['entry'])
                waypoints.extend(connection)
        
        # 计算总路径长度
        total_length = 0.0
        for i in range(1, len(waypoints)):
            dx = waypoints[i].pose.position.x - waypoints[i-1].pose.position.x
            dy = waypoints[i].pose.position.y - waypoints[i-1].pose.position.y
            total_length += math.sqrt(dx*dx + dy*dy)
        
        self.get_logger().info('========== 自动全屋覆盖路径规划完成 ==========')
        self.get_logger().info(f'✓ 总航点数: {len(waypoints)}')
        self.get_logger().info(f'✓ 覆盖子区域数: {len(cells)}')
        self.get_logger().info(f'✓ 总路径长度: {total_length:.2f}m')
        
        return waypoints
    
    def _extract_wall_boundary(self) -> np.ndarray:

        # =======================
        # 创建保存目录def _extract_wall_boundary(self) -> np.ndarray:
        """
        提取墙体边界（不偏移）：
        1. 读取栅格地图，二值化
        2. 按照机器尺寸进行膨胀（避免机器人碰墙）
        3. 开运算、闭运算去除噪声
        返回处理后的二值图像
        """
        if self.static_map is None:
            return None
        
        width = self.static_map.info.width
        height = self.static_map.info.height
        resolution = self.static_map.info.resolution
        
        self.get_logger().info('  [地图预处理] 步骤1: 读取地图并二值化...')
        map_data = np.array(self.static_map.data, dtype=np.int8).reshape((height, width))
        # 二值化：0(自由)=255(白), 其他=0(黑)
        binary = np.where(map_data == 0, 0, 255).astype(np.uint8)
        
        self.get_logger().info('  [地图预处理] 步骤2: 按机器尺寸膨胀（避免碰墙）...')
        robot_pixels = int(self.robot_radius / resolution)
        kernel_robot = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (robot_pixels * 2 + 1, robot_pixels * 2 + 1))
        dilated = cv2.dilate(binary, kernel_robot)
        
        self.get_logger().info('  [地图预处理] 步骤3: 开运算和闭运算去除噪声...')
        kernel_morph = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        morphed = cv2.morphologyEx(dilated, cv2.MORPH_OPEN, kernel_morph)
        morphed = cv2.morphologyEx(morphed, cv2.MORPH_CLOSE, kernel_morph)
        
        # =============================
        # 创建保存目录
        # =============================
        save_dir = os.path.expanduser('~/clean_map_process')
        os.makedirs(save_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')

        # =============================
        # 保存处理前的 map_data 地图
        # =============================
        map_data_vis = binary.copy()
        cv2.imwrite(
            os.path.join(save_dir, f'{timestamp}_01_map_data.png'),
            map_data_vis
        )

        self.get_logger().info(
            f'  [地图预处理] 已保存 map_data 地图: {save_dir}'
        )

        # =============================
        # 保存处理前的 dilated 地图
        # =============================
        dilated_vis = dilated.copy()
        cv2.imwrite(
            os.path.join(save_dir, f'{timestamp}_02_dilated.png'),
            dilated_vis
        )
        self.get_logger().info(
            f'  [地图预处理] 已保存 dilated 地图: {save_dir}'
        )

        return morphed
    
    def _preprocess_map(self) -> Tuple[List[Tuple[float, float]], List[List[Tuple[float, float]]]]:
        """
        地图预处理（提取清扫区域边界和障碍物）：
        1. 提取墙体边界
        2. 向内偏移edge_offset形成清扫区域
        3. findContours找到所有多边形轮廓
        4. 最大面积的作为清扫区域边界，其他的都是障碍物
        """
        morphed = self._extract_wall_boundary()
        if morphed is None:
            return [], []

        width = self.static_map.info.width
        height = self.static_map.info.height
        resolution = self.static_map.info.resolution
        origin_x = self.static_map.info.origin.position.x
        origin_y = self.static_map.info.origin.position.y

        # =============================
        # 创建保存目录
        # =============================
        save_dir = os.path.expanduser('~/clean_map_process')
        os.makedirs(save_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')

        # =============================
        # 保存处理前的 morphed 地图
        # =============================
        morphed_vis = morphed.copy()
        cv2.imwrite(
            os.path.join(save_dir, f'{timestamp}_01_morphed.png'),
            morphed_vis
        )

        self.get_logger().info(
            f'  [地图预处理] 已保存 morphed 地图: {save_dir}'
        )

        # =============================
        # 向内偏移形成清扫区域
        # =============================
        self.get_logger().info(
            f'  [地图预处理] 步骤4: 向内偏移 {self.edge_offset:.2f} m 形成清扫区域...'
        )

        offset_pixels = int(self.edge_offset / resolution)
        kernel_offset = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (offset_pixels * 2 + 1, offset_pixels * 2 + 1)
        )

        # morphed中：黑色=自由空间，白色=墙体
        # 对墙体再次膨胀，向内收缩清扫区域
        cleaning_area_binary = cv2.dilate(morphed, kernel_offset)
        
        # 反转：白色=清扫区域，黑色=墙体+偏移
        cleaning_area = cv2.bitwise_not(cleaning_area_binary)

        # 保存清扫区域
        cv2.imwrite(
            os.path.join(save_dir, f'{timestamp}_02_cleaning_area.png'),
            cleaning_area
        )

        # =============================
        # 查找多边形轮廓
        # =============================
        self.get_logger().info(
            '  [地图预处理] 步骤5: findContours 找到所有多边形轮廓...'
        )

        contours, hierarchy = cv2.findContours(
            cleaning_area,
            cv2.RETR_CCOMP,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # =============================
        # 绘制并标注轮廓
        # =============================
        # 转为彩色图，方便画彩色轮廓
        contour_vis = cv2.cvtColor(cleaning_area, cv2.COLOR_GRAY2BGR)

        for idx, cnt in enumerate(contours):
            # 外轮廓：绿色，内轮廓（障碍物洞）：红色
            color = (0, 255, 0)
            if hierarchy is not None and hierarchy[0][idx][3] != -1:
                color = (0, 0, 255)

            cv2.drawContours(
                contour_vis,
                contours,
                idx,
                color,
                thickness=2
            )

            # 标注轮廓编号
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.putText(
                    contour_vis,
                    f'{idx}',
                    (cx, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    1,
                    cv2.LINE_AA
                )

        # 保存标注后的轮廓图
        cv2.imwrite(
            os.path.join(save_dir, f'{timestamp}_03_contours_labeled.png'),
            contour_vis
        )

        self.get_logger().info(
            f'  [地图预处理] 已保存轮廓标注图，共检测到 {len(contours)} 个轮廓'
)
        
        if not contours:
            self.get_logger().error('  未找到任何清扫区域轮廓！')
            return [], []
        
        self.get_logger().info(f'  找到{len(contours)}个轮廓')
        
        # 找到最大面积的轮廓作为清扫区域边界
        areas = [cv2.contourArea(c) for c in contours]
        max_idx = areas.index(max(areas))
        
        # 清扫区域边界
        largest_contour = contours[max_idx]
        # 使用更小的epsilon保留更多边界细节（沿边清扫需要密集点）
        epsilon = 0.001 * cv2.arcLength(largest_contour, True)
        approx_outer = cv2.approxPolyDP(largest_contour, epsilon, True)
        
        # 如果简化后点太少，直接使用原始轮廓点
        if len(approx_outer) < 8:
            self.get_logger().info(f'  简化后顶点过少({len(approx_outer)}个)，使用原始轮廓')
            approx_outer = largest_contour
        
        cleaning_boundary = []
        for point in approx_outer:
            px, py = point[0]
            wx = px * resolution + origin_x
            wy = py * resolution + origin_y
            cleaning_boundary.append((wx, wy))
        
        # 障碍物（其他轮廓，面积较小的）
        obstacles = []
        min_obstacle_area = 10.0  # 最小障碍物面积（像素^2）
        for i, contour in enumerate(contours):
            if i != max_idx and areas[i] >= min_obstacle_area:
                # 使用更小的epsilon保留障碍物细节
                epsilon = 0.005 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                # 如果简化后点太少，使用原始轮廓
                if len(approx) < 4:
                    approx = contour
                
                obstacle = []
                for point in approx:
                    px, py = point[0]
                    wx = px * resolution + origin_x
                    wy = py * resolution + origin_y
                    obstacle.append((wx, wy))
                if len(obstacle) >= 3:  # 至少3个点才构成有效障碍物
                    obstacles.append(obstacle)
        
        self.get_logger().info(f'  ✓ 清扫区域边界: {len(cleaning_boundary)}个顶点, 面积={max(areas):.2f}像素²')
        self.get_logger().info(f'  ✓ 障碍物: {len(obstacles)}个')
        
        # 保存清扫区域mask用于安全检查
        self.cleaning_area_mask = cleaning_area
        
        # 保存边界数据用于可视化
        self.outer_boundary = cleaning_boundary
        self.obstacles = obstacles
        
        return cleaning_boundary, obstacles
    
    def _extract_inset_boundary(self) -> List[Tuple[float, float]]:
        """提取清扫区域边界（已向内偏移edge_offset）"""
        cleaning_boundary, _ = self._preprocess_map()
        return cleaning_boundary
    
    def _cell_decomposition(self, boundary: List[Tuple[float, float]], 
                           obstacles: List[List[Tuple[float, float]]]) -> List[dict]:
        """
        Cell Decomposition（多边形分块）
        使用垂直扫描线方法，将区域分解为多个子区域
        """
        if not boundary:
            return []
        
        # 计算边界框
        xs = [p[0] for p in boundary]
        ys = [p[1] for p in boundary]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        self.get_logger().info(f'  边界框: x[{min_x:.2f}, {max_x:.2f}], y[{min_y:.2f}, {max_y:.2f}]')
        
        # 创建二值掩码
        resolution = self.static_map.info.resolution
        origin_x = self.static_map.info.origin.position.x
        origin_y = self.static_map.info.origin.position.y
        width = self.static_map.info.width
        height = self.static_map.info.height
        
        mask = np.zeros((height, width), dtype=np.uint8)
        
        # 将世界坐标转换为像素坐标
        def world_to_pixel(wx, wy):
            px = int((wx - origin_x) / resolution)
            py = int((wy - origin_y) / resolution)
            return px, py
        
        # 填充外边界区域
        boundary_pixels = np.array([world_to_pixel(wx, wy) for wx, wy in boundary], dtype=np.int32)
        cv2.fillPoly(mask, [boundary_pixels], 255)
        
        # 移除障碍物区域
        for obstacle in obstacles:
            obs_pixels = np.array([world_to_pixel(wx, wy) for wx, wy in obstacle], dtype=np.int32)
            cv2.fillPoly(mask, [obs_pixels], 0)
        
        # 计算合适的分块宽度（约2.5米宽度）
        cell_width_world = 2.5
        cell_width_pixels = int(cell_width_world / resolution)
        
        min_x_pixels, _ = world_to_pixel(min_x, min_y)
        max_x_pixels, _ = world_to_pixel(max_x, max_y)
        
        num_cells = max(1, (max_x_pixels - min_x_pixels) // cell_width_pixels)
        self.get_logger().info(f'  预计分块数量: {num_cells} (每块宽度约{cell_width_world:.2f}m)')
        
        # 生成分块
        cells = []
        for i in range(num_cells):
            x_start = min_x_pixels + i * cell_width_pixels
            x_end = max_x_pixels if i == num_cells - 1 else (min_x_pixels + (i + 1) * cell_width_pixels)
            
            # 找到这个垂直带内的有效y范围
            valid_y_min = height
            valid_y_max = 0
            has_valid = False
            
            for x in range(x_start, x_end):
                for y in range(height):
                    if 0 <= x < width and 0 <= y < height:
                        if mask[y, x] > 0:
                            valid_y_min = min(valid_y_min, y)
                            valid_y_max = max(valid_y_max, y)
                            has_valid = True
            
            if not has_valid:
                continue  # 这个分块没有有效区域
            
            # 转换回世界坐标
            def pixel_to_world(px, py):
                wx = px * resolution + origin_x
                wy = py * resolution + origin_y
                return wx, wy
            
            # 创建矩形Cell
            v1 = pixel_to_world(x_start, valid_y_min)
            v2 = pixel_to_world(x_end, valid_y_min)
            v3 = pixel_to_world(x_end, valid_y_max)
            v4 = pixel_to_world(x_start, valid_y_max)
            
            entry = pixel_to_world(x_start, (valid_y_min + valid_y_max) // 2)
            exit_point = pixel_to_world(x_end, (valid_y_min + valid_y_max) // 2)
            
            cell = {
                'vertices': [v1, v2, v3, v4],
                'entry': entry,
                'exit': exit_point,
                'coverage_path': []
            }
            cells.append(cell)
            
            self.get_logger().debug(
                f'  Cell[{len(cells)-1}]: 入口({entry[0]:.2f},{entry[1]:.2f}) -> '
                f'出口({exit_point[0]:.2f},{exit_point[1]:.2f})')
        
        self.get_logger().info(f'  Cell Decomposition完成: 生成{len(cells)}个有效分块')
        return cells
    
    def _generate_cell_coverage_path(self, cell: dict):
        """为单个Cell生成弓形覆盖路径"""
        if len(cell['vertices']) < 3:
            return
        
        # 找到Cell的边界框
        xs = [v[0] for v in cell['vertices']]
        ys = [v[1] for v in cell['vertices']]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        # 自动选择最优方向：选择较短的边进行扫描
        width = max_x - min_x
        height = max_y - min_y
        horizontal = width > height  # True: 水平扫描, False: 垂直扫描
        
        # 采样间隔（固定0.3m）
        sample_interval = 0.3
        stripe_width = self.coverage_stripe_width
        
        coverage_path = []
        
        if horizontal:
            # 水平弓形（沿y方向扫描）
            y = min_y
            direction = 1  # 1: 向右, -1: 向左
            
            while y <= max_y:
                line_points = []
                
                if direction == 1:
                    # 从左到右
                    x = min_x
                    while x <= max_x:
                        # 检查是否在清扫区域内（避开障碍物）
                        if self._is_position_safe(x, y):
                            line_points.append((x, y))
                        x += sample_interval
                else:
                    # 从右到左
                    x = max_x
                    while x >= min_x:
                        if self._is_position_safe(x, y):
                            line_points.append((x, y))
                        x -= sample_interval
                
                # 添加路点
                for px, py in line_points:
                    yaw = 0.0 if direction == 1 else math.pi
                    coverage_path.append(self._create_pose_simple(px, py, yaw))
                
                y += stripe_width
                direction *= -1
        else:
            # 垂直弓形（沿x方向扫描）
            x = min_x
            direction = 1  # 1: 向上, -1: 向下
            
            while x <= max_x:
                line_points = []
                
                if direction == 1:
                    # 从下到上
                    y = min_y
                    while y <= max_y:
                        if self._is_position_safe(x, y):
                            line_points.append((x, y))
                        y += sample_interval
                else:
                    # 从上到下
                    y = max_y
                    while y >= min_y:
                        if self._is_position_safe(x, y):
                            line_points.append((x, y))
                        y -= sample_interval
                
                # 添加路点
                for px, py in line_points:
                    yaw = math.pi / 2.0 if direction == 1 else -math.pi / 2.0
                    coverage_path.append(self._create_pose_simple(px, py, yaw))
                
                x += stripe_width
                direction *= -1
        
        cell['coverage_path'] = coverage_path
    
    def _solve_tsp_with_aco(self, cells: List[dict], start_pos: Tuple[float, float]) -> List[int]:
        """
        使用蚁群算法求解TSP问题
        返回最优的Cell访问顺序
        """
        n = len(cells)
        
        if n == 0:
            return []
        if n == 1:
            return [0]
        
        # 蚁群算法参数
        num_ants = 20
        max_iterations = 100
        alpha = 1.0  # 信息素重要程度
        beta = 2.0   # 启发式因子重要程度
        rho = 0.5    # 信息素挥发率
        q = 100.0    # 信息素强度
        
        self.get_logger().info(
            f'  蚁群算法求解TSP: {n}个城市, {num_ants}只蚂蚁, {max_iterations}次迭代')
        
        # 构建距离矩阵（0号节点代表起点）
        distances = np.zeros((n + 1, n + 1))
        
        # 起点到各Cell入口的距离
        for i in range(n):
            dist = math.sqrt(
                (cells[i]['entry'][0] - start_pos[0])**2 + 
                (cells[i]['entry'][1] - start_pos[1])**2)
            distances[0][i + 1] = dist
            distances[i + 1][0] = dist
        
        # Cell之间的距离（i的出口到j的入口）
        for i in range(n):
            for j in range(n):
                if i != j:
                    dist = math.sqrt(
                        (cells[j]['entry'][0] - cells[i]['exit'][0])**2 + 
                        (cells[j]['entry'][1] - cells[i]['exit'][1])**2)
                    distances[i + 1][j + 1] = dist
        
        # 初始化信息素矩阵
        tau_0 = 1.0 / (n * distances[0][1]) if distances[0][1] > 0 else 1.0
        pheromone = np.full((n + 1, n + 1), tau_0)
        
        # 记录最优解
        best_tour = []
        best_length = float('inf')
        
        # 蚁群算法迭代
        for iter in range(max_iterations):
            tours = []
            tour_lengths = []
            
            # 所有蚂蚁构建解
            for ant in range(num_ants):
                visited = [False] * (n + 1)
                tour = [0]  # 从起点开始
                visited[0] = True
                current = 0
                tour_length = 0.0
                
                # 构建完整路径
                for step in range(n):
                    # 计算转移概率
                    probabilities = np.zeros(n + 1)
                    prob_sum = 0.0
                    
                    for next_node in range(1, n + 1):
                        if not visited[next_node]:
                            tau = pheromone[current][next_node]
                            eta = 1.0 / (distances[current][next_node] + 1e-10)
                            probabilities[next_node] = (tau ** alpha) * (eta ** beta)
                            prob_sum += probabilities[next_node]
                    
                    # 轮盘赌选择下一个城市
                    if prob_sum < 1e-10:
                        # 随机选一个未访问的
                        unvisited = [i for i in range(1, n + 1) if not visited[i]]
                        if unvisited:
                            next_node = np.random.choice(unvisited)
                        else:
                            break
                    else:
                        # 归一化概率
                        probabilities /= prob_sum
                        
                        # 轮盘赌
                        rand_val = np.random.random()
                        cumsum = 0.0
                        next_node = 1
                        
                        for node in range(1, n + 1):
                            if not visited[node]:
                                cumsum += probabilities[node]
                                if rand_val <= cumsum:
                                    next_node = node
                                    break
                    
                    tour_length += distances[current][next_node]
                    current = next_node
                    visited[current] = True
                    tour.append(current)
                
                tours.append(tour)
                tour_lengths.append(tour_length)
                
                # 更新最优解
                if tour_length < best_length:
                    best_length = tour_length
                    best_tour = tour
            
            # 信息素挥发
            pheromone *= (1.0 - rho)
            
            # 信息素更新
            for ant in range(num_ants):
                delta_tau = q / (tour_lengths[ant] + 1e-10)
                tour = tours[ant]
                for i in range(len(tour) - 1):
                    from_node = tour[i]
                    to_node = tour[i + 1]
                    pheromone[from_node][to_node] += delta_tau
                    pheromone[to_node][from_node] += delta_tau
            
            # 每10次迭代输出一次
            if (iter + 1) % 10 == 0:
                self.get_logger().debug(
                    f'  迭代[{iter+1}/{max_iterations}]: 当前最优路径长度={best_length:.2f}m')
        
        # 转换为Cell索引（去掉起点0）
        result = [node - 1 for node in best_tour[1:]]
        
        self.get_logger().info(f'  蚁群算法求解完成: 最优路径长度={best_length:.2f}m')
        return result
    
    def _connect_points(self, from_pos: Tuple[float, float], 
                       to_pos: Tuple[float, float]) -> List[PoseStamped]:
        """连接两个点，生成插值路径"""
        waypoints = []
        
        dist = math.sqrt((to_pos[0] - from_pos[0])**2 + (to_pos[1] - from_pos[1])**2)
        num_steps = max(1, int(dist / self.waypoint_spacing))
        
        for i in range(num_steps + 1):
            t = i / num_steps if num_steps > 0 else 1.0
            x = from_pos[0] + t * (to_pos[0] - from_pos[0])
            y = from_pos[1] + t * (to_pos[1] - from_pos[1])
            
            # 检查是否在清扫区域内
            if self._is_position_safe(x, y):
                yaw = math.atan2(to_pos[1] - from_pos[1], to_pos[0] - from_pos[0])
                waypoints.append(self._create_pose_simple(x, y, yaw))
        
        return waypoints
    
    def _interpolate_waypoints(self, x1: float, y1: float, x2: float, y2: float) -> List[PoseStamped]:
        """在两点之间插值生成路点"""
        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if dist < self.waypoint_spacing:
            return []
        
        num_steps = int(dist / self.waypoint_spacing)
        waypoints = []
        
        for i in range(1, num_steps):
            t = i / num_steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            waypoints.append(self._create_pose(x, y, x2, y2))
        
        return waypoints
    
    def _create_pose(self, x: float, y: float, target_x: float, target_y: float) -> PoseStamped:
        """创建位姿（朝向指向目标点）"""
        yaw = math.atan2(target_y - y, target_x - x)
        return self._create_pose_simple(x, y, yaw)
    
    def _create_pose_simple(self, x: float, y: float, yaw: float) -> PoseStamped:
        """创建位姿（直接指定yaw）"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def _is_position_safe(self, x: float, y: float, strict: bool = True) -> bool:
        """
        检查位置是否在清扫区域内（避开障碍物）
        
        Args:
            x, y: 世界坐标
            strict: True=严格检查（弓形/全屋覆盖），False=宽松检查（沿边清扫）
        """
        if self.static_map is None or self.cleaning_area_mask is None:
            return False
        
        resolution = self.static_map.info.resolution
        origin_x = self.static_map.info.origin.position.x
        origin_y = self.static_map.info.origin.position.y
        width = self.static_map.info.width
        height = self.static_map.info.height
        
        # 转换为地图坐标
        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)
        
        # 检查范围
        if mx < 0 or mx >= width or my < 0 or my >= height:
            return False
        
        # 检查该点是否在清扫区域内（mask中为255表示在清扫区域内）
        if self.cleaning_area_mask[my, mx] == 0:
            return False
        
        if strict:
            # 严格模式：检查周围小范围内是否都在清扫区域（用于弓形/全屋覆盖）
            check_radius = 2  # 像素
            for dy in range(-check_radius, check_radius + 1):
                for dx in range(-check_radius, check_radius + 1):
                    cx = mx + dx
                    cy = my + dy
                    
                    if cx < 0 or cx >= width or cy < 0 or cy >= height:
                        return False
                    
                    if self.cleaning_area_mask[cy, cx] == 0:
                        return False
        else:
            # 宽松模式：只检查中心点（用于沿边清扫，允许接近边界）
            # 已经在上面检查过中心点了
            pass
        
        return True
    
    def _is_point_in_polygon(self, x: float, y: float, boundary: List[Tuple[float, float]]) -> bool:
        """检查点是否在多边形内（射线法）"""
        n = len(boundary)
        inside = False
        
        p1x, p1y = boundary[0]
        for i in range(1, n + 1):
            p2x, p2y = boundary[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
    
    def _publish_path(self):
        """发布完整路径（供前端显示）"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = self.waypoints
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'📍 发布完整清扫路径: {len(self.waypoints)}个航点')
        
        # 可视化路点
        self._publish_waypoint_markers()
    
    def _publish_waypoint_markers(self):
        """在RViz中可视化所有路点"""
        marker_array = MarkerArray()
        
        # 路点标记（小球）
        points_marker = Marker()
        points_marker.header.frame_id = 'map'
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = 'waypoints'
        points_marker.id = 0
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.1
        points_marker.scale.y = 0.1
        points_marker.scale.z = 0.1
        points_marker.color.r = 0.0
        points_marker.color.g = 1.0
        points_marker.color.b = 0.0
        points_marker.color.a = 0.8
        
        for waypoint in self.waypoints:
            p = Point()
            p.x = waypoint.pose.position.x
            p.y = waypoint.pose.position.y
            p.z = 0.05
            points_marker.points.append(p)
        
        marker_array.markers.append(points_marker)
        
        # 路径连线
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'path_line'
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 0.0
        line_marker.color.g = 0.8
        line_marker.color.b = 1.0
        line_marker.color.a = 0.6
        
        for waypoint in self.waypoints:
            p = Point()
            p.x = waypoint.pose.position.x
            p.y = waypoint.pose.position.y
            p.z = 0.02
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # 起点标记（大红球）
        if self.waypoints:
            start_marker = Marker()
            start_marker.header.frame_id = 'map'
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = 'start_point'
            start_marker.id = 2
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose = self.waypoints[0].pose
            start_marker.scale.x = 0.3
            start_marker.scale.y = 0.3
            start_marker.scale.z = 0.3
            start_marker.color.r = 1.0
            start_marker.color.g = 0.0
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            marker_array.markers.append(start_marker)
            
            # 终点标记（大蓝球）
            end_marker = Marker()
            end_marker.header.frame_id = 'map'
            end_marker.header.stamp = self.get_clock().now().to_msg()
            end_marker.ns = 'end_point'
            end_marker.id = 3
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose = self.waypoints[-1].pose
            end_marker.scale.x = 0.3
            end_marker.scale.y = 0.3
            end_marker.scale.z = 0.3
            end_marker.color.r = 0.0
            end_marker.color.g = 0.0
            end_marker.color.b = 1.0
            end_marker.color.a = 1.0
            marker_array.markers.append(end_marker)
        
        self.waypoint_markers_pub.publish(marker_array)
        self.get_logger().info(f'🎨 发布路点可视化标记')
        
        # 发布边界可视化
        self._publish_boundary_markers()
    
    def _publish_boundary_markers(self):
        """在RViz中可视化边界（清扫区域边界和障碍物）"""
        marker_array = MarkerArray()
        
        # 清扫区域边界（绿色线框）
        if self.outer_boundary:
            outer_marker = Marker()
            outer_marker.header.frame_id = 'map'
            outer_marker.header.stamp = self.get_clock().now().to_msg()
            outer_marker.ns = 'cleaning_boundary'
            outer_marker.id = 0
            outer_marker.type = Marker.LINE_STRIP
            outer_marker.action = Marker.ADD
            outer_marker.scale.x = 0.08  # 线宽
            outer_marker.color.r = 0.0
            outer_marker.color.g = 1.0
            outer_marker.color.b = 0.0
            outer_marker.color.a = 0.9
            outer_marker.pose.orientation.w = 1.0
            
            for wx, wy in self.outer_boundary:
                p = Point()
                p.x = wx
                p.y = wy
                p.z = 0.1
                outer_marker.points.append(p)
            
            # 闭合线条
            if self.outer_boundary:
                p = Point()
                p.x = self.outer_boundary[0][0]
                p.y = self.outer_boundary[0][1]
                p.z = 0.1
                outer_marker.points.append(p)
            
            marker_array.markers.append(outer_marker)
        
        # 障碍物边界（黄色线框）
        for i, obstacle in enumerate(self.obstacles):
            obs_marker = Marker()
            obs_marker.header.frame_id = 'map'
            obs_marker.header.stamp = self.get_clock().now().to_msg()
            obs_marker.ns = 'obstacles'
            obs_marker.id = i + 1
            obs_marker.type = Marker.LINE_STRIP
            obs_marker.action = Marker.ADD
            obs_marker.scale.x = 0.06
            obs_marker.color.r = 1.0
            obs_marker.color.g = 1.0
            obs_marker.color.b = 0.0
            obs_marker.color.a = 0.9
            obs_marker.pose.orientation.w = 1.0
            
            for wx, wy in obstacle:
                p = Point()
                p.x = wx
                p.y = wy
                p.z = 0.1
                obs_marker.points.append(p)
            
            # 闭合
            if obstacle:
                p = Point()
                p.x = obstacle[0][0]
                p.y = obstacle[0][1]
                p.z = 0.1
                obs_marker.points.append(p)
            
            marker_array.markers.append(obs_marker)
        
        self.boundary_markers_pub.publish(marker_array)
        self.get_logger().info(f'🎨 发布边界可视化: 清扫区域边界+{len(self.obstacles)}个障碍物')
    
    def _send_next_waypoint(self):
        """发送下一个航点"""
        # 检查是否还有航点
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().warn('没有更多航点可发送')
            return
        
        # 检查是否已有活跃目标
        if self.current_goal_handle is not None:
            self.get_logger().warn('已有活跃目标，跳过发送')
            return
        
        # 检查服务可用性
        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('NavigateToPose服务不可用')
            return
        
        # 获取当前航点
        waypoint = self.waypoints[self.current_waypoint_idx]
        waypoint_idx = self.current_waypoint_idx
        
        self.get_logger().info(
            f'  ➤ 发送目标点 [{waypoint_idx + 1}/{len(self.waypoints)}]: '
            f'({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})')
        
        # 创建目标消息
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint
        
        # 异步发送目标
        send_future = self.navigate_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda future, idx=waypoint_idx: self._goal_response_callback(future, idx))
    
    def _goal_response_callback(self, future, waypoint_idx: int):
        """目标响应回调"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f'  ✗ 目标点[{waypoint_idx + 1}]被拒绝')
            # 尝试发送下一个目标点
            self.current_waypoint_idx += 1
            if self.is_cleaning and self.current_waypoint_idx < len(self.waypoints):
                self._send_next_waypoint()
            elif self.current_waypoint_idx >= len(self.waypoints):
                self._on_cleaning_completed()
            return
        
        # 保存当前目标句柄
        self.current_goal_handle = goal_handle
        
        self.get_logger().info(f'  ✓ 目标点[{waypoint_idx + 1}]已接受，开始导航')
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, idx=waypoint_idx: self._result_callback(future, idx))
    
    def _result_callback(self, future, waypoint_idx: int):
        """结果回调"""
        result = future.result()
        
        # 清除当前目标句柄
        self.current_goal_handle = None
        
        if result.status == 4:  # SUCCEEDED
            # 计算进度
            completed = waypoint_idx + 1
            total = len(self.waypoints)
            progress = (completed / total) * 100.0
            
            # 终端输出进度（带进度条）
            bar_length = 40
            filled_length = int(bar_length * completed / total)
            bar = '█' * filled_length + '░' * (bar_length - filled_length)
            
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'  ✓ 完成目标点 [{completed}/{total}]')
            self.get_logger().info(f'  进度: [{bar}] {progress:.1f}%')
            self.get_logger().info(f'  剩余: {total - completed}个航点')
            self.get_logger().info('=' * 60)
            
            # 发布进度
            self._publish_progress(progress)
            
            # 移动到下一个航点
            self.current_waypoint_idx += 1
            
            # 检查是否全部完成
            if self.current_waypoint_idx >= total:
                self._on_cleaning_completed()
            elif self.is_cleaning:
                # 发送下一个目标点
                self._send_next_waypoint()
        else:
            self.get_logger().error(f'  ✗ 目标点[{waypoint_idx + 1}]失败，状态: {result.status}')
            
            # 移动到下一个航点
            self.current_waypoint_idx += 1
            
            # 继续发送下一个目标点（如果还在清扫模式）
            if self.is_cleaning and self.current_waypoint_idx < len(self.waypoints):
                self._send_next_waypoint()
            elif self.current_waypoint_idx >= len(self.waypoints):
                self._on_cleaning_completed()
    
    def _on_cleaning_completed(self):
        """清扫完成"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎉 清扫任务完成！')
        self.get_logger().info('=' * 60)
        
        self.is_cleaning = False
        self._publish_task_info('completed:清扫完成')
        self._publish_progress(100.0)
    
    def _publish_task_info(self, info: str):
        """发布任务信息"""
        msg = String()
        msg.data = info
        self.task_info_pub.publish(msg)
    
    def _publish_progress(self, progress: float):
        """发布清扫进度"""
        msg = Float32()
        msg.data = progress
        self.progress_pub.publish(msg)
        self.get_logger().info(f'📊 清扫进度: {progress:.1f}%')
    
    def _publish_planner_selection(self, planner: str):
        """发布规划器选择"""
        msg = String()
        msg.data = planner
        self.planner_selector_pub.publish(msg)
        self.get_logger().info(f'🔧 选择规划器: {planner}')


def main(args=None):
    rclpy.init(args=args)
    node = CleaningTaskNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

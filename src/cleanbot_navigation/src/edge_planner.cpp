#include "cleanbot_navigation/edge_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <limits>
#include <cmath>
#include <nav_msgs/srv/get_map.hpp>

PLUGINLIB_EXPORT_CLASS(cleanbot_navigation::EdgePlanner, nav2_core::GlobalPlanner)

namespace cleanbot_navigation
{

void EdgePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 调用基类配置
  CoveragePlannerBase::configure(parent, name, tf, costmap_ros);

  auto lifecycle_node = parent.lock();
  if (!lifecycle_node) {
    throw std::runtime_error("Unable to lock node!");
  }

  // 声明沿边偏移参数
  if (!lifecycle_node->has_parameter(name_ + ".edge_offset")) {
    lifecycle_node->declare_parameter(name_ + ".edge_offset", 0.35);
  }
  if (!lifecycle_node->has_parameter(name_ + ".corner_radius")) {
    lifecycle_node->declare_parameter(name_ + ".corner_radius", 0.3);
  }
  if (!lifecycle_node->has_parameter(name_ + ".min_corner_angle")) {
    lifecycle_node->declare_parameter(name_ + ".min_corner_angle", 0.3);
  }

  // 获取参数
  lifecycle_node->get_parameter(name_ + ".edge_offset", edge_offset_);
  lifecycle_node->get_parameter(name_ + ".corner_radius", corner_radius_);
  lifecycle_node->get_parameter(name_ + ".min_corner_angle", min_corner_angle_);

  // 创建普通节点用于订阅静态地图和调用服务
  node_ = std::make_shared<rclcpp::Node>(name_ + "_map_client");
  
  // 订阅静态地图（/map话题，作为备用）
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&EdgePlanner::mapCallback, this, std::placeholders::_1));

  // 创建地图服务客户端（主要方式）
  map_client_ = node_->create_client<nav_msgs::srv::GetMap>("/map_server/map");

  RCLCPP_INFO(
    logger_, "沿边规划器配置完成: edge_offset=%.2fm, corner_radius=%.2fm, min_corner_angle=%.2frad",
    edge_offset_, corner_radius_, min_corner_angle_);
  RCLCPP_INFO(logger_, "使用/map_server/map服务获取静态地图");
}

void EdgePlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  static_map_ = msg;
  RCLCPP_INFO(logger_, "收到静态地图更新: %ux%u, 分辨率=%.3fm", 
    msg->info.width, msg->info.height, msg->info.resolution);
}

bool EdgePlanner::requestStaticMap()
{
  if (!map_client_) {
    RCLCPP_ERROR(logger_, "地图服务客户端未初始化");
    return false;
  }

  // 等待服务可用
  RCLCPP_INFO(logger_, "等待地图服务 /map_server/map ...");
  if (!map_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger_, "地图服务 /map_server/map 不可用！");
    RCLCPP_ERROR(logger_, "请确保启动了 map_server 节点");
    return false;
  }

  // 创建请求
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  
  // 同步调用服务
  RCLCPP_INFO(logger_, "正在请求静态地图...");
  auto future = map_client_->async_send_request(request);
  
  // 等待响应（最多5秒）
  auto spin_node = [this](std::shared_ptr<rclcpp::Node> node) {
    rclcpp::spin_some(node);
  };
  
  auto status = future.wait_for(std::chrono::seconds(5));
  while (status != std::future_status::ready && rclcpp::ok()) {
    spin_node(node_);
    status = future.wait_for(std::chrono::milliseconds(100));
  }
  
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(logger_, "获取静态地图超时");
    return false;
  }

  auto response = future.get();
  if (!response) {
    RCLCPP_ERROR(logger_, "获取静态地图失败：空响应");
    return false;
  }

  // 保存地图
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    static_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(response->map);
    RCLCPP_INFO(logger_, "✓ 成功获取静态地图: %ux%u, 分辨率=%.3fm", 
      static_map_->info.width, static_map_->info.height, static_map_->info.resolution);
  }

  return true;
}

nav_msgs::msg::Path EdgePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  RCLCPP_INFO(
    logger_, "🚀 [%s] 开始规划沿边清扫路径... 起点:(%.2f, %.2f)",
    name_.c_str(), start.pose.position.x, start.pose.position.y);

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  // 检查是否有静态地图，如果没有则主动请求
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!static_map_) {
      RCLCPP_WARN(logger_, "未缓存静态地图，尝试通过服务主动获取...");
    }
  }
  
  // 如果没有地图，主动请求
  if (!static_map_) {
    if (!requestStaticMap()) {
      RCLCPP_ERROR(logger_, "无法获取静态地图，路径规划失败");
      return path;
    }
  }

  // 检查是否取消
  if (cancel_checker && cancel_checker()) {
    RCLCPP_WARN(logger_, "路径规划被取消");
    return path;
  }

  // 调用子类的具体实现
  auto poses = computeCoveragePath(start, goal);
  
  if (poses.empty()) {
    RCLCPP_WARN(logger_, "清扫路径为空！");
    return path;
  }

  // 直接使用原始路径，不进行平滑处理
  path.poses = poses;
  
  RCLCPP_INFO(
    logger_, "✓ 沿边清扫路径规划完成 [%s]: %zu个航点",
    name_.c_str(), path.poses.size());

  return path;
}

std::vector<geometry_msgs::msg::PoseStamped> EdgePlanner::computeCoveragePath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  (void)goal;  // 沿边清扫不需要goal参数
  std::vector<geometry_msgs::msg::PoseStamped> path;

  RCLCPP_INFO(logger_, "========== 沿边清扫路径规划 ==========");
  RCLCPP_INFO(logger_, "[1/4] 从静态地图提取并内偏移边界...");

  // 提取内偏移后的边界
  auto boundary = extractInsetBoundary();
  if (boundary.empty()) {
    RCLCPP_ERROR(logger_, "边界提取失败");
    return path;
  }

  RCLCPP_INFO(logger_, "✓ 内偏移边界提取完成: %zu个顶点, 偏移距离=%.2fm", 
    boundary.size(), edge_offset_);

  // 找到距离起点最近的边界点
  RCLCPP_INFO(logger_, "[2/4] 寻找最近边界点...");
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < boundary.size(); ++i) {
    double dx = boundary[i].first - start.pose.position.x;
    double dy = boundary[i].second - start.pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  
  RCLCPP_INFO(logger_, "✓ 最近边界点索引=%zu, 距离=%.2fm", closest_idx, min_dist);

  // 从起点到最近边界点
  RCLCPP_INFO(logger_, "[3/4] 生成起点到边界的路径...");
  
  // 添加起点
  geometry_msgs::msg::PoseStamped start_pose = start;
  start_pose.header.stamp = clock_->now();
  path.push_back(start_pose);

  // 插值到最近边界点（检查障碍物）
  double dx = boundary[closest_idx].first - start.pose.position.x;
  double dy = boundary[closest_idx].second - start.pose.position.y;
  double dist = std::sqrt(dx*dx + dy*dy);
  int num_steps = std::max(1, static_cast<int>(dist / 0.3));  // 0.3m间距
  
  for (int j = 1; j <= num_steps; ++j) {
    double t = static_cast<double>(j) / (num_steps + 1);
    double x = start.pose.position.x + t * dx;
    double y = start.pose.position.y + t * dy;
    
    // 检查该位置是否安全（无障碍物）
    if (!isPositionSafeInStaticMap(x, y)) {
      RCLCPP_DEBUG(logger_, "跳过不安全点: (%.2f, %.2f)", x, y);
      continue;  // 跳过有障碍物的点
    }
    
    geometry_msgs::msg::PoseStamped interp_pose;
    interp_pose.header.stamp = clock_->now();
    interp_pose.header.frame_id = global_frame_;
    interp_pose.pose.position.x = x;
    interp_pose.pose.position.y = y;
    interp_pose.pose.position.z = 0.0;
    
    double yaw = calculateYaw(
      start.pose.position.x, start.pose.position.y,
      boundary[closest_idx].first, boundary[closest_idx].second);
    interp_pose.pose.orientation.z = std::sin(yaw / 2.0);
    interp_pose.pose.orientation.w = std::cos(yaw / 2.0);
    
    path.push_back(interp_pose);
  }

  // 沿边界走一圈（绕开障碍物）
  RCLCPP_INFO(logger_, "[4/4] 生成沿边清扫路径（绕开障碍物）...");
  
  int skipped_count = 0;  // 统计跳过的不安全点
  
  for (size_t count = 0; count < boundary.size(); ++count) {
    size_t i = (closest_idx + count) % boundary.size();
    
    // 检查当前边界点是否安全
    if (!isPositionSafeInStaticMap(boundary[i].first, boundary[i].second)) {
      RCLCPP_DEBUG(logger_, "边界点不安全，跳过: (%.2f, %.2f)", 
        boundary[i].first, boundary[i].second);
      skipped_count++;
      continue;  // 跳过有障碍物的边界点
    }
    
    // 添加当前边界点
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = clock_->now();
    pose.header.frame_id = global_frame_;
    
    pose.pose.position.x = boundary[i].first;
    pose.pose.position.y = boundary[i].second;
    pose.pose.position.z = 0.0;

    // 计算朝向（指向下一个边界点）
    size_t next_idx = (i + 1) % boundary.size();
    double yaw = calculateYaw(
      boundary[i].first, boundary[i].second,
      boundary[next_idx].first, boundary[next_idx].second);

    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);

    path.push_back(pose);
    
    // 边界点之间插值（0.3m间距，检查障碍物）
    if (count < boundary.size() - 1) {
      double dx = boundary[next_idx].first - boundary[i].first;
      double dy = boundary[next_idx].second - boundary[i].second;
      double dist = std::sqrt(dx*dx + dy*dy);
      
      int num_steps = std::max(1, static_cast<int>(dist / 0.3));
      
      for (int j = 1; j < num_steps; ++j) {
        double t = static_cast<double>(j) / num_steps;
        double x = boundary[i].first + t * dx;
        double y = boundary[i].second + t * dy;
        
        // 检查插值点是否安全
        if (!isPositionSafeInStaticMap(x, y)) {
          RCLCPP_DEBUG(logger_, "插值点不安全，跳过: (%.2f, %.2f)", x, y);
          skipped_count++;
          continue;  // 跳过有障碍物的插值点
        }
        
        geometry_msgs::msg::PoseStamped interp_pose;
        interp_pose.header.stamp = clock_->now();
        interp_pose.header.frame_id = global_frame_;
        interp_pose.pose.position.x = x;
        interp_pose.pose.position.y = y;
        interp_pose.pose.position.z = 0.0;
        interp_pose.pose.orientation = pose.pose.orientation;
        
        path.push_back(interp_pose);
      }
    }
  }
  
  if (skipped_count > 0) {
    RCLCPP_INFO(logger_, "⚠ 跳过了%d个不安全的路径点（有障碍物）", skipped_count);
  }

  RCLCPP_INFO(logger_, "========== 沿边清扫路径规划完成 ==========");
  RCLCPP_INFO(logger_, "✓ 原始航点数: %zu", path.size());
  
  // 对路径进行圆弧平滑处理
  RCLCPP_INFO(logger_, "[5/5] 平滑路径转角（圆弧过渡）...");
  auto smoothed_path = smoothPathWithArcs(path);
  RCLCPP_INFO(logger_, "✓ 平滑后航点数: %zu", smoothed_path.size());
  
  return smoothed_path;
}

std::vector<std::pair<double, double>> EdgePlanner::extractInsetBoundary()
{
  std::vector<std::pair<double, double>> boundary;

  // 获取静态地图
  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!static_map_) {
      RCLCPP_ERROR(logger_, "静态地图未就绪");
      return boundary;
    }
    map = static_map_;
  }

  // 获取地图信息
  unsigned int size_x = map->info.width;
  unsigned int size_y = map->info.height;
  double resolution = map->info.resolution;
  double origin_x = map->info.origin.position.x;
  double origin_y = map->info.origin.position.y;

  RCLCPP_INFO(logger_, "使用静态地图: %ux%u, 分辨率=%.3fm", size_x, size_y, resolution);

  // 步骤1: 创建二值图像
  cv::Mat binary_map(size_y, size_x, CV_8UC1);
  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      int8_t value = map->data[y * size_x + x];
      // OccupancyGrid: -1=未知, 0=自由, 100=障碍物
      // 转换: 未知和障碍物为白色(255), 自由空间为黑色(0)
      if (value == -1 || value > 50) {
        binary_map.at<unsigned char>(y, x) = 255;
      } else {
        binary_map.at<unsigned char>(y, x) = 0;
      }
    }
  }

  // 步骤2: 按照机器人尺寸进行膨胀
  int dilation_pixels = static_cast<int>(robot_radius_ / resolution);
  cv::Mat kernel_dilate = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, 
    cv::Size(dilation_pixels * 2 + 1, dilation_pixels * 2 + 1));
  cv::Mat dilated;
  cv::dilate(binary_map, dilated, kernel_dilate);
  
  RCLCPP_DEBUG(logger_, "膨胀操作完成 (kernel_size=%d)", dilation_pixels * 2 + 1);

  // 步骤3: 开运算和闭运算,去除噪声
  cv::Mat kernel_morph = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::Mat morphed;
  cv::morphologyEx(dilated, morphed, cv::MORPH_OPEN, kernel_morph);   // 开运算
  cv::morphologyEx(morphed, morphed, cv::MORPH_CLOSE, kernel_morph);  // 闭运算
  
  RCLCPP_DEBUG(logger_, "形态学处理完成");

  // 步骤4: 反转图像,找自由空间
  cv::Mat free_space = 255 - morphed;

  // 步骤5: 用findContours找到所有多边形轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(free_space, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  RCLCPP_DEBUG(logger_, "找到轮廓数量: %zu", contours.size());

  if (contours.empty()) {
    RCLCPP_ERROR(logger_, "未找到任何轮廓");
    return boundary;
  }

  // 步骤6: 找到面积最大的轮廓作为外边界
  size_t max_idx = 0;
  double max_area = 0.0;
  for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area > max_area) {
      max_area = area;
      max_idx = i;
    }
  }

  RCLCPP_INFO(logger_, "外边界: 面积=%.2f, 原始顶点数=%zu", 
    max_area, contours[max_idx].size());

  // 步骤7: 边界向内偏移（关键步骤！）
  // 使用腐蚀操作实现内偏移
  int offset_pixels = static_cast<int>(edge_offset_ / resolution);
  
  if (offset_pixels > 0) {
    // 创建包含边界的图像
    cv::Mat boundary_mask = cv::Mat::zeros(size_y, size_x, CV_8UC1);
    std::vector<std::vector<cv::Point>> boundary_vec = {contours[max_idx]};
    cv::fillPoly(boundary_mask, boundary_vec, cv::Scalar(255));
    
    // 腐蚀操作实现内偏移
    cv::Mat kernel_erode = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, 
      cv::Size(offset_pixels * 2 + 1, offset_pixels * 2 + 1));
    cv::Mat eroded;
    cv::erode(boundary_mask, eroded, kernel_erode);
    
    // 提取内偏移后的轮廓
    std::vector<std::vector<cv::Point>> inset_contours;
    cv::findContours(eroded, inset_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (inset_contours.empty()) {
      RCLCPP_WARN(logger_, "内偏移后未找到轮廓，使用原始边界");
      // 使用原始边界
      double epsilon = 0.01 * cv::arcLength(contours[max_idx], true);
      std::vector<cv::Point> approx;
      cv::approxPolyDP(contours[max_idx], approx, epsilon, true);
      
      for (const auto& pt : approx) {
        double wx = pt.x * resolution + origin_x;
        double wy = pt.y * resolution + origin_y;
        boundary.push_back({wx, wy});
      }
    } else {
      // 找到最大的内偏移轮廓
      size_t inset_max_idx = 0;
      double inset_max_area = 0.0;
      for (size_t i = 0; i < inset_contours.size(); ++i) {
        double area = cv::contourArea(inset_contours[i]);
        if (area > inset_max_area) {
          inset_max_area = area;
          inset_max_idx = i;
        }
      }
      
      RCLCPP_INFO(logger_, "内偏移边界: 偏移=%.2fm (%d像素), 面积=%.2f", 
        edge_offset_, offset_pixels, inset_max_area);
      
      // 简化内偏移轮廓
      double epsilon = 0.01 * cv::arcLength(inset_contours[inset_max_idx], true);
      std::vector<cv::Point> approx;
      cv::approxPolyDP(inset_contours[inset_max_idx], approx, epsilon, true);
      
      // 转换为世界坐标
      for (const auto& pt : approx) {
        double wx = pt.x * resolution + origin_x;
        double wy = pt.y * resolution + origin_y;
        boundary.push_back({wx, wy});
      }
    }
  } else {
    // 不进行内偏移
    double epsilon = 0.01 * cv::arcLength(contours[max_idx], true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contours[max_idx], approx, epsilon, true);
    
    for (const auto& pt : approx) {
      double wx = pt.x * resolution + origin_x;
      double wy = pt.y * resolution + origin_y;
      boundary.push_back({wx, wy});
    }
  }

  return boundary;
}

bool EdgePlanner::isPositionSafeInStaticMap(double x, double y)
{
  // 获取静态地图
  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!static_map_) {
      return false;  // 没有地图，认为不安全
    }
    map = static_map_;
  }

  // 将世界坐标转换为地图坐标
  int mx = static_cast<int>((x - map->info.origin.position.x) / map->info.resolution);
  int my = static_cast<int>((y - map->info.origin.position.y) / map->info.resolution);

  // 检查是否在地图范围内
  if (mx < 0 || mx >= static_cast<int>(map->info.width) ||
      my < 0 || my >= static_cast<int>(map->info.height)) {
    return false;  // 超出地图范围
  }

  // 检查机器人半径范围内是否有障碍物
  int check_radius = static_cast<int>(robot_radius_ / map->info.resolution);

  for (int dy = -check_radius; dy <= check_radius; ++dy) {
    for (int dx = -check_radius; dx <= check_radius; ++dx) {
      int cx = mx + dx;
      int cy = my + dy;

      // 检查是否在地图范围内
      if (cx < 0 || cx >= static_cast<int>(map->info.width) ||
          cy < 0 || cy >= static_cast<int>(map->info.height)) {
        return false;  // 超出边界，认为不安全
      }

      // 获取该位置的占用值
      int8_t value = map->data[cy * map->info.width + cx];
      
      // OccupancyGrid: -1=未知, 0=自由, 100=障碍物
      // 如果是未知或障碍物（>50），认为不安全
      if (value == -1 || value > 50) {
        return false;  // 有障碍物或未知区域
      }
    }
  }

  return true;  // 安全
}

std::vector<geometry_msgs::msg::PoseStamped> EdgePlanner::smoothPathWithArcs(
  const std::vector<geometry_msgs::msg::PoseStamped>& path)
{
  std::vector<geometry_msgs::msg::PoseStamped> smoothed;
  
  if (path.size() < 3) {
    return path;  // 路径太短，无需平滑
  }

  int smoothed_corners = 0;
  
  // 添加第一个点
  smoothed.push_back(path[0]);

  // 处理中间点
  for (size_t i = 1; i < path.size() - 1; ++i) {
    const auto& prev = path[i - 1];
    const auto& curr = path[i];
    const auto& next = path[i + 1];

    // 计算前后向量
    double dx1 = curr.pose.position.x - prev.pose.position.x;
    double dy1 = curr.pose.position.y - prev.pose.position.y;
    double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    
    double dx2 = next.pose.position.x - curr.pose.position.x;
    double dy2 = next.pose.position.y - curr.pose.position.y;
    double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

    // 如果距离太短，直接添加原点
    if (len1 < 1e-6 || len2 < 1e-6) {
      smoothed.push_back(curr);
      continue;
    }

    // 归一化向量
    dx1 /= len1; dy1 /= len1;
    dx2 /= len2; dy2 /= len2;

    // 计算转角
    double dot = dx1 * dx2 + dy1 * dy2;
    dot = std::max(-1.0, std::min(1.0, dot));  // 限制在[-1,1]
    double angle = std::acos(dot);

    // 如果转角太小，直接添加原点
    if (angle < min_corner_angle_) {
      smoothed.push_back(curr);
      continue;
    }

    // 计算圆弧平滑参数
    // 根据转角大小动态调整圆弧半径
    double radius = corner_radius_;
    if (angle > M_PI / 2.0) {  // 大于90度的转角
      radius = corner_radius_ * 1.5;  // 增大半径
    }
    
    // 计算从转角点退后的距离
    double tan_half = std::tan(angle / 2.0);
    if (tan_half < 1e-6) {
      smoothed.push_back(curr);
      continue;
    }
    
    double offset = std::min(radius / tan_half, std::min(len1, len2) * 0.4);

    // 圆弧起点（在curr之前）
    double start_x = curr.pose.position.x - dx1 * offset;
    double start_y = curr.pose.position.y - dy1 * offset;
    
    // 圆弧终点（在curr之后）
    double end_x = curr.pose.position.x + dx2 * offset;
    double end_y = curr.pose.position.y + dy2 * offset;

    // 添加圆弧起点
    geometry_msgs::msg::PoseStamped arc_start;
    arc_start.header = curr.header;
    arc_start.pose.position.x = start_x;
    arc_start.pose.position.y = start_y;
    arc_start.pose.position.z = 0.0;
    
    double yaw_start = std::atan2(dy1, dx1);
    arc_start.pose.orientation.z = std::sin(yaw_start / 2.0);
    arc_start.pose.orientation.w = std::cos(yaw_start / 2.0);
    
    smoothed.push_back(arc_start);

    // 生成圆弧插值点
    // 圆弧点数根据转角大小决定
    int num_arc_points = std::max(3, static_cast<int>(angle / 0.2));  // 每0.2弧度一个点
    
    for (int j = 1; j < num_arc_points; ++j) {
      double t = static_cast<double>(j) / num_arc_points;
      double arc_angle = angle * t;
      
      // 计算旋转后的方向向量
      double cos_a = std::cos(arc_angle);
      double sin_a = std::sin(arc_angle);
      
      // 2D旋转矩阵
      double rotated_dx = dx1 * cos_a - dy1 * sin_a;
      double rotated_dy = dx1 * sin_a + dy1 * cos_a;
      
      geometry_msgs::msg::PoseStamped arc_point;
      arc_point.header = curr.header;
      
      // 圆弧路径点
      arc_point.pose.position.x = start_x + offset * (rotated_dx - dx1 + (1 - cos_a) * dx1 + sin_a * dy1);
      arc_point.pose.position.y = start_y + offset * (rotated_dy - dy1 + (1 - cos_a) * dy1 - sin_a * dx1);
      arc_point.pose.position.z = 0.0;
      
      // 切线方向（一阶导数连续）
      double yaw_arc = std::atan2(rotated_dy, rotated_dx);
      arc_point.pose.orientation.z = std::sin(yaw_arc / 2.0);
      arc_point.pose.orientation.w = std::cos(yaw_arc / 2.0);
      
      // 检查圆弧点是否安全
      if (isPositionSafeInStaticMap(arc_point.pose.position.x, arc_point.pose.position.y)) {
        smoothed.push_back(arc_point);
      }
    }

    // 添加圆弧终点
    geometry_msgs::msg::PoseStamped arc_end;
    arc_end.header = curr.header;
    arc_end.pose.position.x = end_x;
    arc_end.pose.position.y = end_y;
    arc_end.pose.position.z = 0.0;
    
    double yaw_end = std::atan2(dy2, dx2);
    arc_end.pose.orientation.z = std::sin(yaw_end / 2.0);
    arc_end.pose.orientation.w = std::cos(yaw_end / 2.0);
    
    smoothed.push_back(arc_end);
    
    smoothed_corners++;
  }

  // 添加最后一个点
  smoothed.push_back(path.back());
  
  RCLCPP_INFO(logger_, "圆弧平滑: 处理了%d个转角, 原始点数=%zu, 平滑后点数=%zu",
    smoothed_corners, path.size(), smoothed.size());

  return smoothed;
}

}  // namespace cleanbot_navigation

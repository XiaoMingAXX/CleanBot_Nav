#include "cleanbot_navigation/coverage_planner_base.hpp"
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace cleanbot_navigation
{

void CoveragePlannerBase::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  // 声明参数
  if (!node->has_parameter(name_ + ".robot_radius")) {
    node->declare_parameter(name_ + ".robot_radius", 0.15);
  }
  if (!node->has_parameter(name_ + ".boundary_margin")) {
    node->declare_parameter(name_ + ".boundary_margin", 0.35);
  }
  if (!node->has_parameter(name_ + ".waypoint_spacing")) {
    node->declare_parameter(name_ + ".waypoint_spacing", 0.5);
  }

  // 获取参数
  node->get_parameter(name_ + ".robot_radius", robot_radius_);
  node->get_parameter(name_ + ".boundary_margin", boundary_margin_);
  node->get_parameter(name_ + ".waypoint_spacing", waypoint_spacing_);

  RCLCPP_INFO(
    logger_, "配置清扫规划器 [%s]: robot_radius=%.2f, boundary_margin=%.2f",
    name_.c_str(), robot_radius_, boundary_margin_);
}

void CoveragePlannerBase::cleanup()
{
  RCLCPP_INFO(logger_, "清理清扫规划器 [%s]", name_.c_str());
}

void CoveragePlannerBase::activate()
{
  RCLCPP_INFO(logger_, "激活清扫规划器 [%s]", name_.c_str());
}

void CoveragePlannerBase::deactivate()
{
  RCLCPP_INFO(logger_, "停用清扫规划器 [%s]", name_.c_str());
}

nav_msgs::msg::Path CoveragePlannerBase::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  RCLCPP_INFO(
    logger_, "🚀 [%s] 开始规划清扫路径... 起点:(%.2f, %.2f) 目标:(%.2f, %.2f)",
    name_.c_str(), start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

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

  RCLCPP_INFO(logger_, "原始路径: %zu个航点", poses.size());

  // 直接使用原始路径，不进行平滑处理
  // 平滑算法会导致路径变形，让DWB直接跟踪原始路径
  path.poses = poses;
  
  RCLCPP_INFO(
    logger_, "✓ 清扫路径规划完成 [%s]: %zu个航点",
    name_.c_str(), path.poses.size());

  return path;
}

std::vector<std::pair<double, double>> CoveragePlannerBase::extractBoundary()
{
  std::vector<std::pair<double, double>> boundary;

  // 获取costmap信息
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  double resolution = costmap_->getResolution();
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();

  // 创建二值图像
  cv::Mat binary_map(size_y, size_x, CV_8UC1);
  
  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      unsigned char cost = costmap_->getCost(x, y);
      binary_map.at<unsigned char>(y, x) = (cost >= 253) ? 255 : 0;
    }
  }

  // 膨胀（扩大障碍物）
  int margin_pixels = static_cast<int>(boundary_margin_ / resolution);
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(margin_pixels * 2 + 1, margin_pixels * 2 + 1));
  cv::Mat dilated;
  cv::dilate(binary_map, dilated, kernel);

  // 反转找空闲区域
  cv::Mat free_space = 255 - dilated;

  // 寻找轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(free_space, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    RCLCPP_WARN(logger_, "未找到有效边界");
    return boundary;
  }

  // 选择最大轮廓
  auto largest_contour = *std::max_element(
    contours.begin(), contours.end(),
    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
      return cv::contourArea(a) < cv::contourArea(b);
    });

  // 简化轮廓
  std::vector<cv::Point> approx;
  double epsilon = 0.01 * cv::arcLength(largest_contour, true);
  cv::approxPolyDP(largest_contour, approx, epsilon, true);

  // 转换为世界坐标
  for (const auto& pt : approx) {
    double wx = pt.x * resolution + origin_x;
    double wy = pt.y * resolution + origin_y;
    boundary.push_back({wx, wy});
  }

  RCLCPP_INFO(logger_, "边界提取完成: %zu个点", boundary.size());
  return boundary;
}

double CoveragePlannerBase::calculateYaw(double x1, double y1, double x2, double y2)
{
  return std::atan2(y2 - y1, x2 - x1);
}

bool CoveragePlannerBase::isPointInPolygon(
  double x, double y,
  const std::vector<std::pair<double, double>>& polygon)
{
  int n = polygon.size();
  bool inside = false;

  double p1x = polygon[0].first;
  double p1y = polygon[0].second;

  for (int i = 1; i <= n; i++) {
    double p2x = polygon[i % n].first;
    double p2y = polygon[i % n].second;

    if (y > std::min(p1y, p2y)) {
      if (y <= std::max(p1y, p2y)) {
        if (x <= std::max(p1x, p2x)) {
          if (p1y != p2y) {
            double xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
            if (p1x == p2x || x <= xinters) {
              inside = !inside;
            }
          }
        }
      }
    }
    p1x = p2x;
    p1y = p2y;
  }

  return inside;
}

bool CoveragePlannerBase::isPositionSafe(double x, double y)
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return false;
  }

  int check_radius = static_cast<int>(robot_radius_ / costmap_->getResolution());

  for (int dy = -check_radius; dy <= check_radius; ++dy) {
    for (int dx = -check_radius; dx <= check_radius; ++dx) {
      unsigned int cx = mx + dx;
      unsigned int cy = my + dy;

      if (cx >= costmap_->getSizeInCellsX() || cy >= costmap_->getSizeInCellsY()) {
        return false;
      }

      unsigned char cost = costmap_->getCost(cx, cy);
      if (cost >= 253) {
        return false;
      }
    }
  }

  return true;
}

std::vector<geometry_msgs::msg::PoseStamped> CoveragePlannerBase::smoothPath(
  const std::vector<geometry_msgs::msg::PoseStamped>& original_path,
  double corner_radius)
{
  std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
  
  if (original_path.size() < 3) {
    return original_path;  // 少于3个点，无需平滑
  }

  // 机器人运动学约束参数（差分驱动）
  const double max_curvature = 1.0 / 0.5;  // 最小转弯半径0.5米
  const double max_angular_change = M_PI / 8.0;  // 相邻点最大角度变化22.5度

  // 添加起点
  smoothed_path.push_back(original_path[0]);

  // 处理中间点
  for (size_t i = 1; i < original_path.size() - 1; ++i) {
    const auto& prev = original_path[i - 1];
    const auto& curr = original_path[i];
    const auto& next = original_path[i + 1];

    // 计算前后向量
    double dx1 = curr.pose.position.x - prev.pose.position.x;
    double dy1 = curr.pose.position.y - prev.pose.position.y;
    double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
    
    double dx2 = next.pose.position.x - curr.pose.position.x;
    double dy2 = next.pose.position.y - curr.pose.position.y;
    double len2 = std::sqrt(dx2*dx2 + dy2*dy2);

    if (len1 < 1e-6 || len2 < 1e-6) {
      smoothed_path.push_back(curr);
      continue;
    }

    // 归一化向量
    dx1 /= len1; dy1 /= len1;
    dx2 /= len2; dy2 /= len2;

    // 计算转角
    double dot = dx1*dx2 + dy1*dy2;
    double angle = std::acos(std::max(-1.0, std::min(1.0, dot)));

    // 如果转角小于15度，直接添加原点
    if (angle < M_PI / 12.0) {
      smoothed_path.push_back(curr);
      continue;
    }

    // 【关键改进1】根据转角大小动态调整圆角半径，确保可跟踪
    // 转角越大，圆角半径越大，拉开距离
    double dynamic_radius = corner_radius;
    if (angle > M_PI / 3.0) {  // 大于60度
      dynamic_radius = corner_radius * 2.0;  // 加倍
    } else if (angle > M_PI / 6.0) {  // 大于30度
      dynamic_radius = corner_radius * 1.5;
    }
    
    double radius = std::min(dynamic_radius, std::min(len1, len2) * 0.45);
    double tan_half = std::tan(angle / 2.0);
    double offset = std::min(radius / tan_half, std::min(len1, len2) * 0.48);

    // 【关键改进2】在转角前添加"减速缓冲区"
    double buffer_distance = offset * 0.5;
    geometry_msgs::msg::PoseStamped buffer_before;
    buffer_before.header = curr.header;
    buffer_before.pose.position.x = curr.pose.position.x - dx1 * (offset + buffer_distance);
    buffer_before.pose.position.y = curr.pose.position.y - dy1 * (offset + buffer_distance);
    buffer_before.pose.position.z = 0.0;
    
    double yaw_buffer = calculateYaw(
      prev.pose.position.x, prev.pose.position.y,
      buffer_before.pose.position.x, buffer_before.pose.position.y);
    buffer_before.pose.orientation.z = std::sin(yaw_buffer / 2.0);
    buffer_before.pose.orientation.w = std::cos(yaw_buffer / 2.0);
    
    smoothed_path.push_back(buffer_before);

    // 圆角前的点
    geometry_msgs::msg::PoseStamped before_corner;
    before_corner.header = curr.header;
    before_corner.pose.position.x = curr.pose.position.x - dx1 * offset;
    before_corner.pose.position.y = curr.pose.position.y - dy1 * offset;
    before_corner.pose.position.z = 0.0;
    
    double yaw_before = calculateYaw(
      buffer_before.pose.position.x, buffer_before.pose.position.y,
      before_corner.pose.position.x, before_corner.pose.position.y);
    before_corner.pose.orientation.z = std::sin(yaw_before / 2.0);
    before_corner.pose.orientation.w = std::cos(yaw_before / 2.0);
    
    smoothed_path.push_back(before_corner);

    // 【关键改进3】使用圆弧而非贝塞尔曲线，保证曲率一致性
    // 计算圆弧中心和半径
    int num_arc_points = std::max(5, static_cast<int>(angle / max_angular_change));
    
    // 计算圆弧中心（两条垂直平分线的交点）
    double mid1x = before_corner.pose.position.x;
    double mid1y = before_corner.pose.position.y;
    double mid2x = curr.pose.position.x + dx2 * offset;
    double mid2y = curr.pose.position.y + dy2 * offset;
    
    // 使用圆弧插值
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
      
      // 沿圆弧路径
      arc_point.pose.position.x = before_corner.pose.position.x + 
                                   offset * (rotated_dx - dx1) / std::sin(angle/2.0);
      arc_point.pose.position.y = before_corner.pose.position.y + 
                                   offset * (rotated_dy - dy1) / std::sin(angle/2.0);
      arc_point.pose.position.z = 0.0;
      
      // 切线方向
      double yaw_arc = std::atan2(rotated_dy, rotated_dx);
      arc_point.pose.orientation.z = std::sin(yaw_arc / 2.0);
      arc_point.pose.orientation.w = std::cos(yaw_arc / 2.0);
      
      smoothed_path.push_back(arc_point);
    }

    // 圆角后的点
    geometry_msgs::msg::PoseStamped after_corner;
    after_corner.header = curr.header;
    after_corner.pose.position.x = curr.pose.position.x + dx2 * offset;
    after_corner.pose.position.y = curr.pose.position.y + dy2 * offset;
    after_corner.pose.position.z = 0.0;
    
    double yaw_after = calculateYaw(
      after_corner.pose.position.x, after_corner.pose.position.y,
      next.pose.position.x, next.pose.position.y);
    after_corner.pose.orientation.z = std::sin(yaw_after / 2.0);
    after_corner.pose.orientation.w = std::cos(yaw_after / 2.0);
    
    smoothed_path.push_back(after_corner);

    // 【关键改进4】在转角后添加"加速缓冲区"
    geometry_msgs::msg::PoseStamped buffer_after;
    buffer_after.header = curr.header;
    buffer_after.pose.position.x = curr.pose.position.x + dx2 * (offset + buffer_distance);
    buffer_after.pose.position.y = curr.pose.position.y + dy2 * (offset + buffer_distance);
    buffer_after.pose.position.z = 0.0;
    buffer_after.pose.orientation = after_corner.pose.orientation;
    
    smoothed_path.push_back(buffer_after);
  }

  // 添加终点
  smoothed_path.push_back(original_path.back());

  return smoothed_path;
}

}  // namespace cleanbot_navigation


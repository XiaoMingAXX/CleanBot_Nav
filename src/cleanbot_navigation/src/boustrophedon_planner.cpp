#include "cleanbot_navigation/boustrophedon_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(cleanbot_navigation::BoustrophedonPlanner, nav2_core::GlobalPlanner)

namespace cleanbot_navigation
{

std::vector<geometry_msgs::msg::PoseStamped> BoustrophedonPlanner::computeCoveragePath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<geometry_msgs::msg::PoseStamped> path;

  // 提取边界
  auto boundary = extractBoundary();
  if (boundary.empty()) {
    RCLCPP_ERROR(logger_, "边界提取失败");
    return path;
  }

  // 计算边界框
  double min_x = boundary[0].first, max_x = boundary[0].first;
  double min_y = boundary[0].second, max_y = boundary[0].second;
  
  for (const auto& pt : boundary) {
    min_x = std::min(min_x, pt.first);
    max_x = std::max(max_x, pt.first);
    min_y = std::min(min_y, pt.second);
    max_y = std::max(max_y, pt.second);
  }

  // 弓字形间距（机器人直径）
  double stripe_width = robot_radius_ * 2.0;
  double sample_spacing = costmap_->getResolution() * 5.0;

  // 生成弓字形路径
  double y = min_y;
  int direction = 1;  // 1: 左到右, -1: 右到左

  while (y <= max_y) {
    std::vector<std::pair<double, double>> line_points;

    if (direction == 1) {
      for (double x = min_x; x <= max_x; x += sample_spacing) {
        if (isPointInPolygon(x, y, boundary) && isPositionSafe(x, y)) {
          line_points.push_back({x, y});
        }
      }
    } else {
      for (double x = max_x; x >= min_x; x -= sample_spacing) {
        if (isPointInPolygon(x, y, boundary) && isPositionSafe(x, y)) {
          line_points.push_back({x, y});
        }
      }
    }

    // 添加这条线到路径
    for (size_t i = 0; i < line_points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = clock_->now();
      pose.header.frame_id = global_frame_;
      
      pose.pose.position.x = line_points[i].first;
      pose.pose.position.y = line_points[i].second;
      pose.pose.position.z = 0.0;

      // 计算朝向
      double yaw = 0.0;
      if (i < line_points.size() - 1) {
        yaw = calculateYaw(
          line_points[i].first, line_points[i].second,
          line_points[i+1].first, line_points[i+1].second);
      } else if (i > 0) {
        yaw = calculateYaw(
          line_points[i-1].first, line_points[i-1].second,
          line_points[i].first, line_points[i].second);
      }

      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);

      path.push_back(pose);
    }

    y += stripe_width;
    direction *= -1;
  }

  RCLCPP_INFO(logger_, "弓形路径规划完成: %zu个航点", path.size());
  return path;
}

}  // namespace cleanbot_navigation


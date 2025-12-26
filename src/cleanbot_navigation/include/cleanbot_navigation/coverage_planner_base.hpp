#ifndef CLEANBOT_NAVIGATION__COVERAGE_PLANNER_BASE_HPP_
#define CLEANBOT_NAVIGATION__COVERAGE_PLANNER_BASE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace cleanbot_navigation
{

/**
 * @class CoveragePlannerBase
 * @brief 清扫路径规划器基类
 */
class CoveragePlannerBase : public nav2_core::GlobalPlanner
{
public:
  CoveragePlannerBase() = default;
  virtual ~CoveragePlannerBase() = default;

  /**
   * @brief 配置插件
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 清理插件
   */
  void cleanup() override;

  /**
   * @brief 激活插件
   */
  void activate() override;

  /**
   * @brief 停用插件
   */
  void deactivate() override;

  /**
   * @brief 创建路径
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  /**
   * @brief 具体的路径规划实现（子类实现）
   */
  virtual std::vector<geometry_msgs::msg::PoseStamped> computeCoveragePath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;

  /**
   * @brief 提取地图边界点
   */
  std::vector<std::pair<double, double>> extractBoundary();

  /**
   * @brief 计算两点间的朝向
   */
  double calculateYaw(double x1, double y1, double x2, double y2);

  /**
   * @brief 检查位置是否在多边形内
   */
  bool isPointInPolygon(
    double x, double y, 
    const std::vector<std::pair<double, double>>& polygon);

  /**
   * @brief 检查位置是否安全（无障碍物）
   */
  bool isPositionSafe(double x, double y);

  /**
   * @brief 平滑路径，将尖锐转角圆角化
   */
  std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& original_path,
    double corner_radius = 0.3);

  // 成员变量
  rclcpp::Logger logger_{rclcpp::get_logger("CoveragePlannerBase")};
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;

  // 参数
  double robot_radius_{0.15};
  double boundary_margin_{0.35};
  double waypoint_spacing_{0.5};
};

}  // namespace cleanbot_navigation

#endif  // CLEANBOT_NAVIGATION__COVERAGE_PLANNER_BASE_HPP_


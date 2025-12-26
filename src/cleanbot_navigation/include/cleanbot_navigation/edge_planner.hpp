#ifndef CLEANBOT_NAVIGATION__EDGE_PLANNER_HPP_
#define CLEANBOT_NAVIGATION__EDGE_PLANNER_HPP_

#include "cleanbot_navigation/coverage_planner_base.hpp"
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>

namespace cleanbot_navigation
{

/**
 * @class EdgePlanner
 * @brief 沿边清扫路径规划器
 * 
 * 使用SLAM的静态地图进行路径规划，而不是动态的costmap
 * 
 * 算法流程：
 * 1. 从/map话题获取SLAM静态地图
 * 2. 二值化，去除噪声
 * 3. 按照机器尺寸进行膨胀
 * 4. 开运算、闭运算处理
 * 5. 用OpenCV的findContours找到所有多边形轮廓
 * 6. 面积最大的作为外边界
 * 7. 将外边界向内偏移edge_offset_距离（默认35cm）作为清扫路径
 */
class EdgePlanner : public CoveragePlannerBase
{
public:
  EdgePlanner() = default;
  ~EdgePlanner() override = default;

  /**
   * @brief 配置插件
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 覆盖createPlan
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  /**
   * @brief 计算沿边清扫路径
   */
  std::vector<geometry_msgs::msg::PoseStamped> computeCoveragePath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /**
   * @brief 提取边界并进行内偏移（使用静态地图）
   * @return 内偏移后的边界点集
   */
  std::vector<std::pair<double, double>> extractInsetBoundary();

  /**
   * @brief 静态地图回调函数
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief 主动获取静态地图（通过service）
   */
  bool requestStaticMap();

  /**
   * @brief 检查位置在静态地图中是否安全（无障碍物）
   * @param x 世界坐标x
   * @param y 世界坐标y
   * @return true表示安全，false表示有障碍物或未知
   */
  bool isPositionSafeInStaticMap(double x, double y);

  /**
   * @brief 平滑路径，将直角转弯改为圆弧过渡
   * @param path 原始路径
   * @return 平滑后的路径
   */
  std::vector<geometry_msgs::msg::PoseStamped> smoothPathWithArcs(
    const std::vector<geometry_msgs::msg::PoseStamped>& path);

  // 边界内偏移距离（米）
  double edge_offset_{0.35};
  
  // 圆弧平滑参数
  double corner_radius_{0.3};  // 转角圆弧半径（米）
  double min_corner_angle_{0.3};  // 最小转角角度（弧度，约17度）才进行平滑
  
  // 静态地图订阅器和缓存
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;
  std::mutex map_mutex_;
  
  // 节点指针（用于创建订阅器和服务客户端）
  rclcpp::Node::SharedPtr node_;
  
  // 地图服务客户端
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;
};

}  // namespace cleanbot_navigation

#endif  // CLEANBOT_NAVIGATION__EDGE_PLANNER_HPP_


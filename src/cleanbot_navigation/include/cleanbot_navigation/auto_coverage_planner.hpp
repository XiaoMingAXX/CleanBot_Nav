#ifndef CLEANBOT_NAVIGATION__AUTO_COVERAGE_PLANNER_HPP_
#define CLEANBOT_NAVIGATION__AUTO_COVERAGE_PLANNER_HPP_

#include "cleanbot_navigation/coverage_planner_base.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <random>

namespace cleanbot_navigation
{

/**
 * @struct Cell
 * @brief 表示分解后的子区域
 */
struct Cell
{
  std::vector<std::pair<double, double>> vertices;  // 子区域顶点
  std::pair<double, double> entry;                  // 入口点
  std::pair<double, double> exit;                   // 出口点
  std::vector<geometry_msgs::msg::PoseStamped> coverage_path;  // 子区域覆盖路径
};

/**
 * @class AutoCoveragePlanner
 * @brief 自动全屋清扫路径规划器
 * 
 * 基于Cell Decomposition + 蚁群算法的全覆盖路径规划
 * 算法流程:
 * 1. 地图预处理: 二值化、膨胀、提取多边形边界和障碍物
 * 2. Cell Decomposition: 使用扫描线算法分解多边形
 * 3. 为每个Cell生成弓形覆盖路径
 * 4. 使用蚁群算法求解TSP问题,优化访问顺序
 * 5. 连接所有子路径生成完整路径
 */
class AutoCoveragePlanner : public CoveragePlannerBase
{
public:
  AutoCoveragePlanner() = default;
  ~AutoCoveragePlanner() override = default;

  /**
   * @brief 配置插件
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

protected:
  /**
   * @brief 计算自动全屋覆盖路径
   */
  std::vector<geometry_msgs::msg::PoseStamped> computeCoveragePath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /**
   * @brief 地图预处理: 提取边界和障碍物多边形
   * @return pair<外边界, 障碍物列表>
   */
  std::pair<std::vector<cv::Point>, std::vector<std::vector<cv::Point>>> 
  preprocessMap();

  /**
   * @brief Cell Decomposition: 多边形分块
   * @param boundary 外边界
   * @param obstacles 障碍物列表
   * @return 分解后的子区域列表
   */
  std::vector<Cell> cellDecomposition(
    const std::vector<cv::Point>& boundary,
    const std::vector<std::vector<cv::Point>>& obstacles);

  /**
   * @brief 为单个Cell生成弓形覆盖路径
   * @param cell 子区域
   * @param direction 覆盖方向 (0: 水平, 1: 垂直)
   */
  void generateCellCoveragePath(Cell& cell, int direction);

  /**
   * @brief 使用蚁群算法求解TSP问题
   * @param cells 所有子区域
   * @param start_pos 起始位置
   * @return 最优访问顺序
   */
  std::vector<int> solveTSPWithACO(
    const std::vector<Cell>& cells,
    const std::pair<double, double>& start_pos);

  /**
   * @brief 计算两点间的欧氏距离
   */
  double euclideanDistance(
    const std::pair<double, double>& p1,
    const std::pair<double, double>& p2);

  /**
   * @brief 连接两个路径点
   */
  std::vector<geometry_msgs::msg::PoseStamped> connectPoints(
    const std::pair<double, double>& from,
    const std::pair<double, double>& to);

  /**
   * @brief 将像素坐标转换为世界坐标
   */
  std::pair<double, double> pixelToWorld(const cv::Point& pixel);

  /**
   * @brief 将世界坐标转换为像素坐标
   */
  cv::Point worldToPixel(const std::pair<double, double>& world);

  // 蚁群算法参数
  int num_ants_{20};              // 蚂蚁数量
  int max_iterations_{100};       // 最大迭代次数
  double alpha_{1.0};             // 信息素重要程度因子
  double beta_{5.0};              // 启发函数重要程度因子
  double rho_{0.5};               // 信息素挥发系数
  double q_{100.0};               // 信息素强度

  // 覆盖参数
  double coverage_stripe_width_{0.25};  // 覆盖条带宽度(机器人直径0.3m, 留重叠)
  int coverage_direction_{0};           // 覆盖方向 (0: 自动选择最优方向)

  // 随机数生成器
  std::random_device rd_;
  std::mt19937 gen_{rd_()};
};

}  // namespace cleanbot_navigation

#endif  // CLEANBOT_NAVIGATION__AUTO_COVERAGE_PLANNER_HPP_

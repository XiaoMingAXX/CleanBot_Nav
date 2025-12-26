#ifndef CLEANBOT_NAVIGATION__BOUSTROPHEDON_PLANNER_HPP_
#define CLEANBOT_NAVIGATION__BOUSTROPHEDON_PLANNER_HPP_

#include "cleanbot_navigation/coverage_planner_base.hpp"

namespace cleanbot_navigation
{

/**
 * @class BoustrophedonPlanner
 * @brief 弓字形清扫路径规划器
 */
class BoustrophedonPlanner : public CoveragePlannerBase
{
public:
  BoustrophedonPlanner() = default;
  ~BoustrophedonPlanner() override = default;

protected:
  /**
   * @brief 计算弓字形清扫路径
   */
  std::vector<geometry_msgs::msg::PoseStamped> computeCoveragePath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
};

}  // namespace cleanbot_navigation

#endif  // CLEANBOT_NAVIGATION__BOUSTROPHEDON_PLANNER_HPP_


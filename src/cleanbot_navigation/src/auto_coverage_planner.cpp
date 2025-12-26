#include "cleanbot_navigation/auto_coverage_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

PLUGINLIB_EXPORT_CLASS(cleanbot_navigation::AutoCoveragePlanner, nav2_core::GlobalPlanner)

namespace cleanbot_navigation
{

void AutoCoveragePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 调用基类配置
  CoveragePlannerBase::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  // 声明蚁群算法参数
  if (!node->has_parameter(name_ + ".num_ants")) {
    node->declare_parameter(name_ + ".num_ants", 20);
  }
  if (!node->has_parameter(name_ + ".max_iterations")) {
    node->declare_parameter(name_ + ".max_iterations", 100);
  }
  if (!node->has_parameter(name_ + ".coverage_stripe_width")) {
    node->declare_parameter(name_ + ".coverage_stripe_width", 0.25);
  }

  // 获取参数
  node->get_parameter(name_ + ".num_ants", num_ants_);
  node->get_parameter(name_ + ".max_iterations", max_iterations_);
  node->get_parameter(name_ + ".coverage_stripe_width", coverage_stripe_width_);

  RCLCPP_INFO(
    logger_, "自动全屋清扫规划器配置完成: ants=%d, iterations=%d, stripe_width=%.2fm",
    num_ants_, max_iterations_, coverage_stripe_width_);
}

std::vector<geometry_msgs::msg::PoseStamped> AutoCoveragePlanner::computeCoveragePath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  (void)goal;  // 全屋覆盖不需要使用goal参数，消除编译警告
  std::vector<geometry_msgs::msg::PoseStamped> path;

  RCLCPP_INFO(logger_, "========== 开始自动全屋覆盖路径规划 ==========");

  // 步骤1: 地图预处理,提取边界和障碍物
  RCLCPP_INFO(logger_, "[1/5] 地图预处理: 提取多边形边界和障碍物...");
  auto [boundary, obstacles] = preprocessMap();
  
  if (boundary.empty()) {
    RCLCPP_ERROR(logger_, "地图预处理失败: 未找到有效边界");
    return path;
  }
  
  RCLCPP_INFO(logger_, "✓ 提取到外边界: %zu个顶点, 障碍物: %zu个",
    boundary.size(), obstacles.size());

  // 步骤2: Cell Decomposition - 多边形分块
  RCLCPP_INFO(logger_, "[2/5] 执行Cell Decomposition分块算法...");
  auto cells = cellDecomposition(boundary, obstacles);
  
  if (cells.empty()) {
    RCLCPP_ERROR(logger_, "Cell Decomposition失败: 未生成有效子区域");
    return path;
  }
  
  RCLCPP_INFO(logger_, "✓ 分解完成: 共%zu个子区域", cells.size());

  // 步骤3: 为每个Cell生成覆盖路径
  RCLCPP_INFO(logger_, "[3/5] 为每个子区域生成弓形覆盖路径...");
  for (size_t i = 0; i < cells.size(); ++i) {
    generateCellCoveragePath(cells[i], coverage_direction_);
    RCLCPP_DEBUG(logger_, "  子区域[%zu]: %zu个航点", 
      i, cells[i].coverage_path.size());
  }
  RCLCPP_INFO(logger_, "✓ 所有子区域路径生成完成");

  // 步骤4: 使用蚁群算法求解TSP,优化访问顺序
  RCLCPP_INFO(logger_, "[4/5] 使用蚁群算法求解TSP,优化子区域访问顺序...");
  std::pair<double, double> start_pos = {start.pose.position.x, start.pose.position.y};
  auto visit_order = solveTSPWithACO(cells, start_pos);
  
  RCLCPP_INFO(logger_, "✓ TSP求解完成, 最优访问顺序: [");
  std::string order_str;
  for (int idx : visit_order) {
    order_str += std::to_string(idx) + " ";
  }
  RCLCPP_INFO(logger_, "  %s]", order_str.c_str());

  // 步骤5: 按照最优顺序连接所有子路径
  RCLCPP_INFO(logger_, "[5/5] 连接所有子路径生成完整覆盖路径...");
  
  // 从起点到第一个Cell的入口
  if (!visit_order.empty()) {
    auto connection = connectPoints(start_pos, cells[visit_order[0]].entry);
    path.insert(path.end(), connection.begin(), connection.end());
  }

  // 遍历所有Cell
  for (size_t i = 0; i < visit_order.size(); ++i) {
    int cell_idx = visit_order[i];
    const auto& cell = cells[cell_idx];
    
    // 添加Cell内部的覆盖路径
    path.insert(path.end(), cell.coverage_path.begin(), cell.coverage_path.end());
    
    // 如果不是最后一个Cell,连接到下一个Cell
    if (i < visit_order.size() - 1) {
      int next_cell_idx = visit_order[i + 1];
      auto connection = connectPoints(cell.exit, cells[next_cell_idx].entry);
      path.insert(path.end(), connection.begin(), connection.end());
    }
  }

  RCLCPP_INFO(logger_, "========== 自动全屋覆盖路径规划完成 ==========");
  RCLCPP_INFO(logger_, "✓ 总航点数: %zu", path.size());
  RCLCPP_INFO(logger_, "✓ 覆盖子区域数: %zu", cells.size());
  
  // 计算总路径长度
  double total_length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    double dx = path[i].pose.position.x - path[i-1].pose.position.x;
    double dy = path[i].pose.position.y - path[i-1].pose.position.y;
    total_length += std::sqrt(dx*dx + dy*dy);
  }
  RCLCPP_INFO(logger_, "✓ 总路径长度: %.2fm", total_length);

  return path;
}

std::pair<std::vector<cv::Point>, std::vector<std::vector<cv::Point>>> 
AutoCoveragePlanner::preprocessMap()
{
  std::vector<cv::Point> boundary;
  std::vector<std::vector<cv::Point>> obstacles;

  // 获取costmap信息
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  double resolution = costmap_->getResolution();

  RCLCPP_INFO(logger_, "地图尺寸: %ux%u, 分辨率: %.3fm", size_x, size_y, resolution);

  // 步骤1: 创建二值图像
  cv::Mat binary_map(size_y, size_x, CV_8UC1);
  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      unsigned char cost = costmap_->getCost(x, y);
      // 障碍物为白色(255), 自由空间为黑色(0)
      binary_map.at<unsigned char>(y, x) = (cost >= 253) ? 255 : 0;
    }
  }

  // 步骤2: 按照机器人尺寸进行膨胀 (机器人直径0.3m)
  int dilation_pixels = static_cast<int>((robot_radius_ + 0.05) / resolution);
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

  // 步骤5: 使用findContours找到所有多边形轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(free_space, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  
  RCLCPP_INFO(logger_, "找到轮廓数量: %zu", contours.size());

  if (contours.empty()) {
    RCLCPP_ERROR(logger_, "未找到任何轮廓");
    return {boundary, obstacles};
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

  // 简化外边界轮廓
  double epsilon = 0.01 * cv::arcLength(contours[max_idx], true);
  cv::approxPolyDP(contours[max_idx], boundary, epsilon, true);
  
  RCLCPP_INFO(logger_, "外边界: 面积=%.2f, 顶点数=%zu", max_area, boundary.size());

  // 步骤7: 其他轮廓作为障碍物
  double min_obstacle_area = 10.0;  // 最小障碍物面积(像素^2)
  for (size_t i = 0; i < contours.size(); ++i) {
    if (i != max_idx) {
      double area = cv::contourArea(contours[i]);
      if (area >= min_obstacle_area) {
        std::vector<cv::Point> obstacle;
        double eps = 0.02 * cv::arcLength(contours[i], true);
        cv::approxPolyDP(contours[i], obstacle, eps, true);
        obstacles.push_back(obstacle);
        RCLCPP_DEBUG(logger_, "障碍物[%zu]: 面积=%.2f, 顶点数=%zu", 
          obstacles.size()-1, area, obstacle.size());
      }
    }
  }

  return {boundary, obstacles};
}

std::vector<Cell> AutoCoveragePlanner::cellDecomposition(
  const std::vector<cv::Point>& boundary,
  const std::vector<std::vector<cv::Point>>& obstacles)
{
  std::vector<Cell> cells;

  // 简化版Cell Decomposition: 基于垂直扫描线
  // 找到边界框
  int min_x = boundary[0].x, max_x = boundary[0].x;
  int min_y = boundary[0].y, max_y = boundary[0].y;
  
  for (const auto& pt : boundary) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }

  RCLCPP_INFO(logger_, "边界框: x[%d,%d], y[%d,%d]", min_x, max_x, min_y, max_y);

  // 创建二值掩码用于检测
  cv::Mat mask = cv::Mat::zeros(
    costmap_->getSizeInCellsY(), 
    costmap_->getSizeInCellsX(), 
    CV_8UC1);
  
  // 填充外边界区域
  std::vector<std::vector<cv::Point>> boundary_vec = {boundary};
  cv::fillPoly(mask, boundary_vec, cv::Scalar(255));
  
  // 移除障碍物区域
  for (const auto& obstacle : obstacles) {
    std::vector<std::vector<cv::Point>> obs_vec = {obstacle};
    cv::fillPoly(mask, obs_vec, cv::Scalar(0));
  }

  // 使用垂直扫描线进行分块
  // 计算合适的分块宽度 (约2-3米宽度)
  double cell_width_world = 2.5;  // 世界坐标,米
  int cell_width_pixels = static_cast<int>(cell_width_world / costmap_->getResolution());
  
  int num_cells = std::max(1, (max_x - min_x) / cell_width_pixels);
  RCLCPP_INFO(logger_, "预计分块数量: %d (每块宽度约%.2fm)", num_cells, cell_width_world);

  // 生成分块
  for (int i = 0; i < num_cells; ++i) {
    int x_start = min_x + i * cell_width_pixels;
    int x_end = (i == num_cells - 1) ? max_x : (min_x + (i + 1) * cell_width_pixels);
    
    // 找到这个垂直带内的有效区域
    Cell cell;
    
    // 简化: 使用矩形区域作为Cell
    std::vector<cv::Point> cell_vertices;
    
    // 找到这个x范围内的y有效范围
    int valid_y_min = max_y;
    int valid_y_max = min_y;
    bool has_valid = false;
    
    for (int x = x_start; x < x_end; ++x) {
      for (int y = min_y; y < max_y; ++y) {
        if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows) {
          if (mask.at<unsigned char>(y, x) > 0) {
            valid_y_min = std::min(valid_y_min, y);
            valid_y_max = std::max(valid_y_max, y);
            has_valid = true;
          }
        }
      }
    }
    
    if (!has_valid) {
      continue;  // 这个分块没有有效区域
    }
    
    // 创建矩形Cell
    cell.vertices.push_back(pixelToWorld(cv::Point(x_start, valid_y_min)));
    cell.vertices.push_back(pixelToWorld(cv::Point(x_end, valid_y_min)));
    cell.vertices.push_back(pixelToWorld(cv::Point(x_end, valid_y_max)));
    cell.vertices.push_back(pixelToWorld(cv::Point(x_start, valid_y_max)));
    
    // 入口: 左侧中点
    cell.entry = pixelToWorld(cv::Point(x_start, (valid_y_min + valid_y_max) / 2));
    
    // 出口: 右侧中点
    cell.exit = pixelToWorld(cv::Point(x_end, (valid_y_min + valid_y_max) / 2));
    
    cells.push_back(cell);
    
    RCLCPP_DEBUG(logger_, "Cell[%d]: 入口(%.2f,%.2f) -> 出口(%.2f,%.2f)",
      static_cast<int>(cells.size())-1,
      cell.entry.first, cell.entry.second,
      cell.exit.first, cell.exit.second);
  }

  RCLCPP_INFO(logger_, "Cell Decomposition完成: 生成%zu个有效分块", cells.size());
  
  return cells;
}

void AutoCoveragePlanner::generateCellCoveragePath(Cell& cell, int direction)
{
  cell.coverage_path.clear();

  if (cell.vertices.size() < 3) {
    return;
  }

  // 找到Cell的边界框
  double min_x = cell.vertices[0].first;
  double max_x = cell.vertices[0].first;
  double min_y = cell.vertices[0].second;
  double max_y = cell.vertices[0].second;
  
  for (const auto& v : cell.vertices) {
    min_x = std::min(min_x, v.first);
    max_x = std::max(max_x, v.first);
    min_y = std::min(min_y, v.second);
    max_y = std::max(max_y, v.second);
  }

  // 根据方向生成弓形路径
  // direction = 0: 水平方向 (沿y扫描)
  // direction = 1: 垂直方向 (沿x扫描)
  
  // 自动选择最优方向: 选择较短的方向进行扫描
  int actual_direction = direction;
  if (direction == 0) {
    double width = max_x - min_x;
    double height = max_y - min_y;
    actual_direction = (width > height) ? 0 : 1;  // 选择短边方向
  }

  // 使用合理的采样间隔，避免路径点过密
  // 对于清扫任务，0.3m的间隔足够了（机器人直径0.3m）
  double sample_interval = 0.3;  // 固定间隔，避免过密
  
  if (actual_direction == 0) {
    // 水平弓形 (沿y方向扫描)
    double y = min_y;
    int line_direction = 1;  // 1: 向右, -1: 向左
    
    while (y <= max_y) {
      std::vector<geometry_msgs::msg::PoseStamped> line_poses;
      
      if (line_direction == 1) {
        // 从左到右
        for (double x = min_x; x <= max_x; x += sample_interval) {
          if (isPositionSafe(x, y)) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = clock_->now();
            pose.header.frame_id = global_frame_;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;
            
            double yaw = (line_direction == 1) ? 0.0 : M_PI;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            
            line_poses.push_back(pose);
          }
        }
      } else {
        // 从右到左
        for (double x = max_x; x >= min_x; x -= sample_interval) {
          if (isPositionSafe(x, y)) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = clock_->now();
            pose.header.frame_id = global_frame_;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;
            
            double yaw = (line_direction == 1) ? 0.0 : M_PI;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            
            line_poses.push_back(pose);
          }
        }
      }
      
      // 添加这一行的路径点
      cell.coverage_path.insert(
        cell.coverage_path.end(), 
        line_poses.begin(), 
        line_poses.end());
      
      y += coverage_stripe_width_;
      line_direction *= -1;  // 换方向
    }
  } else {
    // 垂直弓形 (沿x方向扫描)
    double x = min_x;
    int line_direction = 1;  // 1: 向上, -1: 向下
    
    while (x <= max_x) {
      std::vector<geometry_msgs::msg::PoseStamped> line_poses;
      
      if (line_direction == 1) {
        // 从下到上
        for (double y = min_y; y <= max_y; y += sample_interval) {
          if (isPositionSafe(x, y)) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = clock_->now();
            pose.header.frame_id = global_frame_;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;
            
            double yaw = (line_direction == 1) ? M_PI/2.0 : -M_PI/2.0;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            
            line_poses.push_back(pose);
          }
        }
      } else {
        // 从上到下
        for (double y = max_y; y >= min_y; y -= sample_interval) {
          if (isPositionSafe(x, y)) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = clock_->now();
            pose.header.frame_id = global_frame_;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;
            
            double yaw = (line_direction == 1) ? M_PI/2.0 : -M_PI/2.0;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            
            line_poses.push_back(pose);
          }
        }
      }
      
      // 添加这一列的路径点
      cell.coverage_path.insert(
        cell.coverage_path.end(), 
        line_poses.begin(), 
        line_poses.end());
      
      x += coverage_stripe_width_;
      line_direction *= -1;  // 换方向
    }
  }
}

std::vector<int> AutoCoveragePlanner::solveTSPWithACO(
  const std::vector<Cell>& cells,
  const std::pair<double, double>& start_pos)
{
  int n = cells.size();
  
  if (n == 0) {
    return {};
  }
  
  if (n == 1) {
    return {0};
  }

  RCLCPP_INFO(logger_, "蚁群算法求解TSP: %d个城市, %d只蚂蚁, %d次迭代",
    n, num_ants_, max_iterations_);

  // 构建距离矩阵 (从起点到各Cell入口, 以及Cell出口到其他Cell入口)
  std::vector<std::vector<double>> distances(n + 1, std::vector<double>(n + 1, 0.0));
  
  // 0号节点代表起点
  for (int i = 0; i < n; ++i) {
    distances[0][i + 1] = euclideanDistance(start_pos, cells[i].entry);
    distances[i + 1][0] = distances[0][i + 1];
  }
  
  // Cell之间的距离 (i的出口到j的入口)
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (i != j) {
        distances[i + 1][j + 1] = euclideanDistance(cells[i].exit, cells[j].entry);
      }
    }
  }

  // 初始化信息素矩阵
  double tau_0 = 1.0 / (n * distances[0][1]);  // 初始信息素
  std::vector<std::vector<double>> pheromone(n + 1, std::vector<double>(n + 1, tau_0));

  // 记录最优解
  std::vector<int> best_tour;
  double best_length = std::numeric_limits<double>::max();

  // 蚁群算法迭代
  for (int iter = 0; iter < max_iterations_; ++iter) {
    // 所有蚂蚁构建解
    std::vector<std::vector<int>> tours(num_ants_);
    std::vector<double> tour_lengths(num_ants_, 0.0);
    
    for (int ant = 0; ant < num_ants_; ++ant) {
      std::vector<bool> visited(n + 1, false);
      std::vector<int>& tour = tours[ant];
      
      int current = 0;  // 从起点开始
      visited[0] = true;
      tour.push_back(0);
      
      // 构建完整路径
      for (int step = 0; step < n; ++step) {
        // 计算转移概率
        std::vector<double> probabilities(n + 1, 0.0);
        double sum = 0.0;
        
        for (int next = 1; next <= n; ++next) {
          if (!visited[next]) {
            double tau = pheromone[current][next];
            double eta = 1.0 / (distances[current][next] + 1e-10);
            probabilities[next] = std::pow(tau, alpha_) * std::pow(eta, beta_);
            sum += probabilities[next];
          }
        }
        
        // 轮盘赌选择下一个城市
        if (sum < 1e-10) {
          // 如果没有有效选择,随机选一个未访问的
          std::vector<int> unvisited;
          for (int i = 1; i <= n; ++i) {
            if (!visited[i]) {
              unvisited.push_back(i);
            }
          }
          if (!unvisited.empty()) {
            std::uniform_int_distribution<> dis(0, unvisited.size() - 1);
            current = unvisited[dis(gen_)];
          }
        } else {
          // 归一化概率
          for (int i = 1; i <= n; ++i) {
            probabilities[i] /= sum;
          }
          
          // 轮盘赌
          std::uniform_real_distribution<> dis(0.0, 1.0);
          double rand_val = dis(gen_);
          double cumsum = 0.0;
          
          for (int next = 1; next <= n; ++next) {
            if (!visited[next]) {
              cumsum += probabilities[next];
              if (rand_val <= cumsum) {
                tour_lengths[ant] += distances[current][next];
                current = next;
                break;
              }
            }
          }
        }
        
        visited[current] = true;
        tour.push_back(current);
      }
      
      // 更新最优解
      if (tour_lengths[ant] < best_length) {
        best_length = tour_lengths[ant];
        best_tour = tour;
      }
    }
    
    // 信息素挥发
    for (int i = 0; i <= n; ++i) {
      for (int j = 0; j <= n; ++j) {
        pheromone[i][j] *= (1.0 - rho_);
      }
    }
    
    // 信息素更新
    for (int ant = 0; ant < num_ants_; ++ant) {
      double delta_tau = q_ / (tour_lengths[ant] + 1e-10);
      for (size_t i = 0; i < tours[ant].size() - 1; ++i) {
        int from = tours[ant][i];
        int to = tours[ant][i + 1];
        pheromone[from][to] += delta_tau;
        pheromone[to][from] += delta_tau;
      }
    }
    
    // 每10次迭代输出一次
    if ((iter + 1) % 10 == 0) {
      RCLCPP_DEBUG(logger_, "迭代[%d/%d]: 当前最优路径长度=%.2fm",
        iter + 1, max_iterations_, best_length);
    }
  }

  // 转换为Cell索引 (去掉起点0)
  std::vector<int> result;
  for (size_t i = 1; i < best_tour.size(); ++i) {
    result.push_back(best_tour[i] - 1);
  }

  RCLCPP_INFO(logger_, "蚁群算法求解完成: 最优路径长度=%.2fm", best_length);
  
  return result;
}

double AutoCoveragePlanner::euclideanDistance(
  const std::pair<double, double>& p1,
  const std::pair<double, double>& p2)
{
  double dx = p2.first - p1.first;
  double dy = p2.second - p1.second;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<geometry_msgs::msg::PoseStamped> AutoCoveragePlanner::connectPoints(
  const std::pair<double, double>& from,
  const std::pair<double, double>& to)
{
  std::vector<geometry_msgs::msg::PoseStamped> path;
  
  double dist = euclideanDistance(from, to);
  int num_steps = std::max(1, static_cast<int>(dist / waypoint_spacing_));
  
  for (int i = 0; i <= num_steps; ++i) {
    double t = static_cast<double>(i) / num_steps;
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = clock_->now();
    pose.header.frame_id = global_frame_;
    
    pose.pose.position.x = from.first + t * (to.first - from.first);
    pose.pose.position.y = from.second + t * (to.second - from.second);
    pose.pose.position.z = 0.0;
    
    double yaw = calculateYaw(from.first, from.second, to.first, to.second);
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    
    path.push_back(pose);
  }
  
  return path;
}

std::pair<double, double> AutoCoveragePlanner::pixelToWorld(const cv::Point& pixel)
{
  double resolution = costmap_->getResolution();
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  
  double wx = pixel.x * resolution + origin_x;
  double wy = pixel.y * resolution + origin_y;
  
  return {wx, wy};
}

cv::Point AutoCoveragePlanner::worldToPixel(const std::pair<double, double>& world)
{
  double resolution = costmap_->getResolution();
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  
  int px = static_cast<int>((world.first - origin_x) / resolution);
  int py = static_cast<int>((world.second - origin_y) / resolution);
  
  return cv::Point(px, py);
}

}  // namespace cleanbot_navigation

#include "cleanbot_control/cleanbot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace cleanbot_control
{

hardware_interface::CallbackReturn CleanBotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 获取关节信息
  joint_names_.resize(info_.joints.size());
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_velocity_.resize(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    joint_names_[i] = info_.joints[i].name;
  }

  // 获取轮子参数
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CleanBotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 创建ROS2节点用于通讯
  node_ = rclcpp::Node::make_shared("cleanbot_hardware_interface_node");
  
  // 订阅joint_states话题（来自USB通讯节点）
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&CleanBotHardwareInterface::joint_state_callback, this, std::placeholders::_1));
  
  // 发布轮速命令话题（给USB通讯节点），单位：m/s
  wheel_speed_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed_cmd", 10);

  RCLCPP_INFO(node_->get_logger(), "CleanBot硬件接口配置完成");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
CleanBotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CleanBotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn CleanBotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "CleanBot硬件接口激活");
  
  // 初始化命令为0
  for (size_t i = 0; i < hw_commands_velocity_.size(); i++)
  {
    hw_commands_velocity_[i] = 0.0;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CleanBotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "CleanBot硬件接口停用");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CleanBotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 处理ROS消息
  rclcpp::spin_some(node_);
  
  // hw_positions_ 和 hw_velocities_ 在joint_state_callback中更新
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CleanBotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 将关节速度命令（rad/s）转换为轮子线速度（m/s）
  // 公式: v = ω × r
  // 假设: joint 0 = left_wheel, joint 1 = right_wheel
  
  if (hw_commands_velocity_.size() >= 2)
  {
    // 控制器下发的是角速度 (rad/s)
    double left_wheel_angular_vel = hw_commands_velocity_[0];
    double right_wheel_angular_vel = hw_commands_velocity_[1];
    
    // 转换为线速度 (m/s): v = ω × r
    double left_wheel_linear_vel = left_wheel_angular_vel * wheel_radius_;
    double right_wheel_linear_vel = right_wheel_angular_vel * wheel_radius_;
    
    // 发布轮速命令（m/s）给USB节点
    auto wheel_speed_msg = std_msgs::msg::Float32MultiArray();
    wheel_speed_msg.data.resize(2);
    wheel_speed_msg.data[0] = static_cast<float>(left_wheel_linear_vel);
    wheel_speed_msg.data[1] = static_cast<float>(right_wheel_linear_vel);
    
    wheel_speed_cmd_pub_->publish(wheel_speed_msg);
  }
  
  return hardware_interface::return_type::OK;
}

void CleanBotHardwareInterface::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 更新关节状态
  for (size_t i = 0; i < msg->name.size(); i++)
  {
    for (size_t j = 0; j < joint_names_.size(); j++)
    {
      if (msg->name[i] == joint_names_[j])
      {
        if (i < msg->position.size())
        {
          hw_positions_[j] = msg->position[i];
        }
        if (i < msg->velocity.size())
        {
          hw_velocities_[j] = msg->velocity[i];
        }
        break;
      }
    }
  }
}

}  // namespace cleanbot_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cleanbot_control::CleanBotHardwareInterface, hardware_interface::SystemInterface)



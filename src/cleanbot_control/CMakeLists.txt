cmake_minimum_required(VERSION 3.8)
project(cleanbot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 可选依赖：ros2_control
find_package(controller_manager QUIET)
find_package(hardware_interface QUIET)
find_package(pluginlib QUIET)
find_package(rclcpp_lifecycle QUIET)

# 如果找到ros2_control依赖，则编译硬件接口
if(hardware_interface_FOUND AND pluginlib_FOUND AND rclcpp_lifecycle_FOUND)
  message(STATUS "找到ros2_control依赖，将编译硬件接口")
  
  # 硬件接口库
  add_library(${PROJECT_NAME} SHARED
    src/cleanbot_hardware_interface.cpp
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
    sensor_msgs
    geometry_msgs
  )

  pluginlib_export_plugin_description_file(hardware_interface cleanbot_control.xml)

  # Install C++ library
  install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )
else()
  message(WARNING "未找到ros2_control依赖，将跳过硬件接口编译。")
  message(WARNING "请安装: sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers")
endif()

# Install Python scripts
install(PROGRAMS
  scripts/usb_communication_node.py
  scripts/web_control_node.py
  scripts/laser_frame_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install web files
install(DIRECTORY
  web
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY
  world
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

ament_package()

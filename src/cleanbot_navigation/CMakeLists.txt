cmake_minimum_required(VERSION 3.8)
project(cleanbot_navigation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  nav2_core
  nav2_util
  nav2_costmap_2d
  nav2_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  pluginlib
  OpenCV
)

# Coverage Planner Base
add_library(cleanbot_coverage_planners SHARED
  src/coverage_planner_base.cpp
  src/edge_planner.cpp
  src/boustrophedon_planner.cpp
  src/auto_coverage_planner.cpp
)

ament_target_dependencies(cleanbot_coverage_planners ${dependencies})
target_link_libraries(cleanbot_coverage_planners ${OpenCV_LIBS})

# Export plugin
pluginlib_export_plugin_description_file(nav2_core coverage_planners_plugin.xml)

# Install C++ libraries
install(TARGETS cleanbot_coverage_planners
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  cleanbot_navigation/navigation_mode_manager.py
  cleanbot_navigation/cleaning_task_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install map files
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(cleanbot_coverage_planners)
ament_export_dependencies(${dependencies})

ament_package()


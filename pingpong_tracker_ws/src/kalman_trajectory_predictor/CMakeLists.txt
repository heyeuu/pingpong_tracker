cmake_minimum_required(VERSION 3.8)
project(kalman_trajectory_predictor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/predictor_node.cpp
  src/image_processor.cpp
  src/kalman_filter.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  OpenCV
  rclcpp_components
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "kalman_trajectory_predictor::PredictorNode"
)

add_executable(image_publisher_node
  src/image_publisher_node.cpp
)

ament_target_dependencies(image_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  image_publisher_node
  DESTINATION lib
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  OpenCV
  rclcpp_components
)

# ament_export_dependencies(rclcpp sensor_msgs cv_bridge geometry_msgs  OpenCV)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

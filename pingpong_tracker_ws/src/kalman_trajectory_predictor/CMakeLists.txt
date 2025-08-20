cmake_minimum_required(VERSION 3.8)
project(kalman_trajectory_predictor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(predictor_node
  src/predictor_node.cpp
  src/kalman_filter.cpp
  )

  add_executable(image_publisher_node
  src/image_publisher_node.cpp
)

ament_target_dependencies(predictor_node 
  rclcpp 
  sensor_msgs 
  cv_bridge
  geometry_msgs 
  OpenCV)

ament_target_dependencies(image_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

#target_link_libraries(predictor_node ${OpenCV_LIBRARIES})

install(TARGETS
  predictor_node
  image_publisher_node
  DESTINATION  lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_dependencies(rclcpp sensor_msgs cv_bridge geometry_msgs OpenCV)

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

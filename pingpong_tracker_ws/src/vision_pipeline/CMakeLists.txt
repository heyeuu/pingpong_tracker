cmake_minimum_required(VERSION 3.8)
project(vision_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find all necessary ROS 2 and third-party dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

# 创建一个共享库用于 Deep Learning 组件。
add_library(deep_learning_component SHARED
  src/deep_learning_component.cpp
)
# 使用 ament_target_dependencies 链接所有依赖项。
ament_target_dependencies(deep_learning_component
  rclcpp
  rclcpp_components
  cv_bridge
  sensor_msgs
  image_transport
  OpenCV
)
# 设置 C++ 标准。
target_compile_features(deep_learning_component PUBLIC cxx_std_20)
# 设置包含路径。
target_include_directories(deep_learning_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 创建一个共享库用于 Kalman Filter 组件。
add_library(kalman_filter_component SHARED
  src/kalman_filter_component.cpp
)
# 使用 ament_target_dependencies 链接所有依赖项。
ament_target_dependencies(kalman_filter_component
  rclcpp
  rclcpp_components
  cv_bridge
  sensor_msgs
  image_transport
  OpenCV
)
# 设置 C++ 标准。
target_compile_features(kalman_filter_component PUBLIC cxx_std_17)
# 设置包含路径。
target_include_directories(kalman_filter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装所有目标。
install(TARGETS
  deep_learning_component
  kalman_filter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# 安装头文件。
install(DIRECTORY include/
  DESTINATION include
)

# 导出依赖项。
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

# 这告诉 ament 去查找测试包。
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 关键更改：将 ament_package() 移动回这里。
# 这样可以确保它在所有依赖项和目标的定义之后被调用。
ament_package()

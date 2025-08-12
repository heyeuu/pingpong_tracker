cmake_minimum_required(VERSION 3.8)
project(mv_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 确保找到所有必要的依赖项。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

# 创建一个共享库，包含组件。
add_library(mv_camera_driver SHARED
  src/camera_driver_node.cpp
  src/camera_wrapper.cpp
)

# 使用 ament_target_dependencies 将所有依赖项的头文件和库链接到目标。
# 这是解决你遇到的问题的最佳实践。
ament_target_dependencies(mv_camera_driver
  rclcpp
  rclcpp_components
  cv_bridge
  sensor_msgs
  image_transport
  OpenCV
)

# 设置C++标准。
target_compile_features(mv_camera_driver PUBLIC cxx_std_17)

# 创建一个独立的调试可执行文件。
add_executable(mv_camera_driver_standalone src/main.cpp)

# 将可执行文件链接到我们创建的库。
target_link_libraries(mv_camera_driver_standalone mv_camera_driver)

# 安装所有目标。
install(TARGETS mv_camera_driver mv_camera_driver_standalone
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# 导出依赖项，供依赖于此包的其他包使用。
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

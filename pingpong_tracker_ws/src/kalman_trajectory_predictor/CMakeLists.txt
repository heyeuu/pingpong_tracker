cmake_minimum_required(VERSION 3.8)
project(kalman_trajectory_predictor)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找所有 ROS2 和系统依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)

# -----------------------------------------------------------------
# 核心：编译功能库 (kalman_trajectory_predictor_core)
# -----------------------------------------------------------------
set(CORE_LIB_SOURCES
  src/lib/ball_detector.cpp
  src/lib/kalman_filter.cpp
  src/lib/coordinate_converter.cpp
  src/lib/pnp_solver.cpp
)

# 创建一个静态库
add_library(${PROJECT_NAME}_core STATIC ${CORE_LIB_SOURCES})

# 设置库的头文件目录
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接核心库的依赖
ament_target_dependencies(${PROJECT_NAME}_core
  OpenCV
  Eigen3 # 这里已经修正了
)

# -----------------------------------------------------------------
# 核心：编译 ROS2 节点可执行文件
# -----------------------------------------------------------------
# 节点列表
set(NODE_EXECUTABLES
  ball_detector_node
  kalman_predictor_node
  image_visualizer_node
)

# 循环创建并链接节点
foreach(NODE_NAME ${NODE_EXECUTABLES})
  add_executable(${NODE_NAME} src/${NODE_NAME}.cpp)

  # 设置节点的头文件目录
  target_include_directories(${NODE_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # 链接节点的 ROS2 依赖
  ament_target_dependencies(${NODE_NAME}
    rclcpp
    sensor_msgs
    cv_bridge
    geometry_msgs
  )

  # 链接我们的核心库
  target_link_libraries(${NODE_NAME}
    ${PROJECT_NAME}_core
  )
endforeach()

# -----------------------------------------------------------------
# 安装和包配置
# -----------------------------------------------------------------
install(DIRECTORY
  include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_core
  ${NODE_EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
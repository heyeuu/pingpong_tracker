cmake_minimum_required(VERSION 3.12)
project(kalman_trajectory_predictor)

# -----------------------------------------------------------------
# 1. 配置项目依赖和编译选项
# -----------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------
# 2. 编译核心库和组件
# -----------------------------------------------------------------

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
  "${PROJECT_SOURCE_DIR}/src/lib/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

# 编译所有节点和核心功能到一个共享库
add_library(${PROJECT_NAME} SHARED
 ${PROJECT_SOURCE}
  src/image_publisher_node.cpp
  src/lib/image_source.cpp
  # 如果你有其他节点，也在这里添加
)

# 设置库的头文件目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接必要的依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  pluginlib
  ament_index_cpp
  camera_info_manager
  image_transport
  cv_bridge
  OpenCV
  Eigen3
)

rclcpp_components_register_node(
  # 第一个参数是你的共享库名
  ${PROJECT_NAME} 
  # 第二个参数是你的节点类
  PLUGIN "kalman_trajectory_predictor::ImagePublisherNode"
  EXECUTABLE ${PROJECT_NAME}_container_executable 
)

# -----------------------------------------------------------------
# 3. 安装和包配置
# -----------------------------------------------------------------
# 安装头文件目录
install(DIRECTORY
  include/
  DESTINATION include
)

install(TARGETS 
${PROJECT_NAME} LIBRARY DESTINATION lib
 ARCHIVE DESTINATION lib)

install(TARGETS
  ${PROJECT_NAME}_container_executable
  # 对于可执行文件
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装 plugins.xml 文件
install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 文件目录
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

# 安装其他资源文件
install(
  DIRECTORY test_video
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
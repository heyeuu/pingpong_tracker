cmake_minimum_required(VERSION 3.12)
project(kalman_trajectory_predictor)

# -----------------------------------------------------------------
# 1. 配置项目依赖和编译选项
# -----------------------------------------------------------------

# 使用 ament_cmake_auto 宏来自动查找依赖
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------
# 2. 编译核心库和组件库
# -----------------------------------------------------------------

# 使用 GLOB_RECURSE 宏来自动查找源文件
# 这个宏会递归地查找 src/lib 目录下的所有 .cpp 文件
file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
  "${PROJECT_SOURCE_DIR}/src/lib/*.cpp"
)

# 编译核心库
# 注意：我们将所有核心功能都编译进一个共享库，以便所有节点共享
# image_source.cpp 包含在内
ament_auto_add_library(
  kalman_trajectory_predictor_core SHARED
  ${PROJECT_SOURCE}
  src/image_publisher_node.cpp
  src/ball_detector_node.cpp
  src/kalman_predictor_node.cpp
  src/image_visualizer_node.cpp
)

# 设置库的头文件目录
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 导出插件描述文件 (Component XML)
# 注意: 'plugins.xml' 文件需要我们手动创建
pluginlib_export_plugin_description_file(${PROJECT_NAME}_core plugins.xml)


# -----------------------------------------------------------------
# 3. 安装和包配置
# -----------------------------------------------------------------
ament_auto_package()
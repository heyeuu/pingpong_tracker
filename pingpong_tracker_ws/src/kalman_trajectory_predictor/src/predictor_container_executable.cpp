// src/predictor_container_executable.cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  // 使用多线程执行器来处理所有节点
  rclcpp::executors::MultiThreadedExecutor executor;
  // 创建一个容器节点，它将作为所有组件的宿主
  auto node = std::make_shared<rclcpp::Node>("predictor_container");
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
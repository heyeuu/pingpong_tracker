// File: src/kalman_filter_component.cpp
#include "vision_pipeline/kalman_filter_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace vision_pipeline {

// 实现构造函数
KalmanFilterComponent::KalmanFilterComponent(const rclcpp::NodeOptions &options)
    : rclcpp::Node("kalman_filter_component", options) {
  RCLCPP_INFO(this->get_logger(),
              "Kalman Filter Component has been initialized.");
}

} // namespace vision_pipeline

// 注册你的组件节点
RCLCPP_COMPONENTS_REGISTER_NODE(vision_pipeline::KalmanFilterComponent)

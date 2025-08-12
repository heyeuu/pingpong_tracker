#include "vision_pipeline/deep_learning_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace vision_pipeline {

// 实现构造函数
DeepLearningComponent::DeepLearningComponent(const rclcpp::NodeOptions &options)
    : rclcpp::Node("deep_learning_component", options) {
  RCLCPP_INFO(this->get_logger(),
              "Deep Learning Component has been initialized.");
  // 示例：创建订阅者
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&DeepLearningComponent::image_callback, this,
                std::placeholders::_1));
}

void DeepLearningComponent::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received image!");
  (void)msg;
}

} // namespace vision_pipeline

// 注册你的组件节点
RCLCPP_COMPONENTS_REGISTER_NODE(vision_pipeline::DeepLearningComponent)
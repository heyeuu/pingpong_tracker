// File: include/vision_pipeline/deep_learning_component.hpp
#ifndef VISION_PIPELINE__DEEP_LEARNING_COMPONENT_HPP_
#define VISION_PIPELINE__DEEP_LEARNING_COMPONENT_HPP_

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// 确保将你的组件类放在 `vision_pipeline` 命名空间中
namespace vision_pipeline {

// 使用 RCLCPP_COMPONENTS_PUBLIC 宏确保类可以在其他包中正确使用
class DeepLearningComponent : public rclcpp::Node {
public:
  // 构造函数，使用 rclcpp::NodeOptions 初始化
  explicit DeepLearningComponent(const rclcpp::NodeOptions &options);

private:
  // 在这里添加你的私有成员变量和方法
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

} // namespace vision_pipeline

#endif // VISION_PIPELINE__DEEP_LEARNING_COMPONENT_HPP_

// File: include/vision_pipeline/kalman_filter_component.hpp
#ifndef VISION_PIPELINE__KALMAN_FILTER_COMPONENT_HPP_
#define VISION_PIPELINE__KALMAN_FILTER_COMPONENT_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

// 确保将你的组件类放在 `vision_pipeline` 命名空间中
namespace vision_pipeline {

// 使用 RCLCPP_COMPONENTS_PUBLIC 宏确保类可以在其他包中正确使用
class KalmanFilterComponent : public rclcpp::Node {
public:
  // 构造函数，使用 rclcpp::NodeOptions 初始化
  explicit KalmanFilterComponent(const rclcpp::NodeOptions &options);

private:
  // 在这里添加你的私有成员变量和方法
  // 例如：cv::KalmanFilter kf;
};

} // namespace vision_pipeline

#endif // VISION_PIPELINE__KALMAN_FILTER_COMPONENT_HPP_

#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "kalman_trajectory_predictor/image_processor.hpp"
#include "kalman_trajectory_predictor/kalman_filter.hpp"
#include "std_msgs/msg/string.hpp"

namespace kalman_trajectory_predictor {

class PredictorNode : public rclcpp::Node {
public:
  explicit PredictorNode(const rclcpp::NodeOptions &options);

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

  KalmanFilter kf_tracker_;
  ImageProcessor image_processor_;

  std::vector<int> hsv_lower_bound_;
  std::vector<int> hsv_upper_bound_;
};

} // namespace kalman_trajectory_predictor

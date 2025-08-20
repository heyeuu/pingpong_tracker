#include "kalman_trajectory_predictor/predictor_node.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

namespace kalman_trajectory_predictor {

PredictorNode::PredictorNode(const rclcpp::NodeOptions &options)
    : Node("kalman_trajectory_predictor", options) {
  this->declare_parameter<std::vector<long int>>("hsv_lower_bound", {0, 0, 0});
  this->declare_parameter<std::vector<long int>>("hsv_upper_bound",
                                                 {255, 255, 255});

  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 10,
      std::bind(&PredictorNode::image_callback, this, std::placeholders::_1));

  publisher_ =
      this->create_publisher<geometry_msgs::msg::Point>("ball_position", 10);
}

void PredictorNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  try {

    auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    std::vector<long int> hsv_lower_bound;
    std::vector<long int> hsv_upper_bound;

    this->get_parameter("hsv_lower_bound", hsv_lower_bound);
    this->get_parameter("hsv_upper_bound", hsv_upper_bound);

    cv::Point ball_position;

    bool ball_found = image_processor_.detectBall(
        frame, ball_position,
        cv::Scalar(static_cast<int>(hsv_lower_bound[0]),
                   static_cast<int>(hsv_lower_bound[1]),
                   static_cast<int>(hsv_lower_bound[2])),
        cv::Scalar(static_cast<int>(hsv_upper_bound[0]),
                   static_cast<int>(hsv_upper_bound[1]),
                   static_cast<int>(hsv_upper_bound[2])));

    if (ball_found) {
      kf_tracker_.update({(float)ball_position.x, (float)ball_position.y});
    }

    cv::Point2f predicted_position_2d = kf_tracker_.predict();

    geometry_msgs::msg::Point predicted_point_msg;
    predicted_point_msg.x = static_cast<double>(predicted_position_2d.x);
    predicted_point_msg.y = static_cast<double>(predicted_position_2d.y);
    predicted_point_msg.z = 0.0;
    publisher_->publish(predicted_point_msg);

  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

} // namespace kalman_trajectory_predictor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kalman_trajectory_predictor::PredictorNode)
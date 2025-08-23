#ifndef KALMAN_TRAJECTORY_PREDICTOR__IMAGE_PUBLISHER_NODE_HPP_
#define KALMAN_TRAJECTORY_PREDICTOR__IMAGE_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace kalman_trajectory_predictor {

class ImagePublisherNode : public rclcpp::Node {
public:
    explicit ImagePublisherNode(const rclcpp::NodeOptions& options);

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::VideoCapture cap_;

    void publishImage();
};

} // namespace kalman_trajectory_predictor

#endif // KALMAN_TRAJECTORY_PREDICTOR__IMAGE_PUBLISHER_NODE_HPP_
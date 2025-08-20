#pragma once

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/opencv.hpp"

#include "kalman_trajectory_predictor/image_processor.hpp"
#include "kalman_trajectory_predictor/kalman_filter.hpp"

class PredictorNode : public rclcpp::Node {
public:
  PredictorNode() : Node("predictor_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10,
        [this](const sensor_msgs::msg::Image::SharedPtr& msg) {
          this->image_callback(msg);
        });
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Point>("ball_position", 10);
    image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr &msg) {
    try {
      auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      cv::Point ball_position;
      bool ball_found = detectBall(cv_ptr->image, ball_position);

      kf_tracker_.predict();

      if (ball_found) {
        kf_tracker_.update(cv::Point3f(static_cast<float>(ball_position.x),
                                       static_cast<float>(ball_position.y),
                                       0.0f));

        cv::circle(cv_ptr->image, ball_position, 5, cv::Scalar(0, 255, 0), -1);
        auto processed_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_ptr->image)
                .toImageMsg();

        image_publisher_->publish(*processed_msg);
      }

      auto filtered_position = kf_tracker_.getFilteredPosition();

      geometry_msgs::msg::Point position_msg;
      position_msg.x = filtered_position.x;
      position_msg.y = filtered_position.y;
      position_msg.z = filtered_position.z;

      publisher_->publish(position_msg);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception:%s", e.what());
    }
  }

  static bool detectBall(const cv::Mat &image, cv::Point &position) {
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Scalar lower_orange(5, 100, 100);
    cv::Scalar upper_orange(15, 255, 255);

    cv::Mat mask;
    cv::inRange(hsv_image, lower_orange, upper_orange, mask);

    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
      double max_area = 0;
      size_t largest_contour_idx = 0;
      for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours.at(i));
        if (area > max_area) {
          max_area = area;
          largest_contour_idx = i;
        }
      }

      if (max_area > 100) {
        auto M = cv::moments(contours[largest_contour_idx]);
        position.x = static_cast<int>(M.m10 / M.m00);
        position.y = static_cast<int>(M.m01 / M.m00);
        return true;
      }
    }
    return false;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

  KalmanFilter kf_tracker_;
};
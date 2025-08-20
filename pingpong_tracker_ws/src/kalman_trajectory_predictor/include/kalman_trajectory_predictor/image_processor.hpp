#pragma once

#include <opencv2/opencv.hpp>

namespace kalman_trajectory_predictor {

class ImageProcessor {
public:
  ImageProcessor() = default;

  static bool detectBall(const cv::Mat &image, cv::Point &ball_position,
                         const cv::Scalar &lower_bound,
                         const cv::Scalar &upper_bound) {
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv_image, lower_bound, upper_bound, mask);

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
        // position.x = static_cast<int>(M.m10 / M.m00);
        // position.y = static_cast<int>(M.m01 / M.m00);
        return true;
      }
    }
    return false;
  };
};

} // namespace kalman_trajectory_predictor
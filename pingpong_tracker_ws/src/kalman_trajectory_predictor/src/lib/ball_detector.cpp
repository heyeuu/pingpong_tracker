#include "kalman_trajectory_predictor/ball_detector.hpp"

#include <opencv2/opencv.hpp>
#include <vector>

namespace kalman_trajectory_predictor {

BallDetector::BallDetector() {
    // 可以在这里进行任何初始化
}

cv::Point2f BallDetector::processImage(const cv::Mat& image) {
    cv::Point2f ball_position(-1.0f, -1.0f); // 默认返回一个无效坐标

    if (image.empty()) {
        return ball_position;
    }

    // 1. 将图像转换为 HSV 颜色空间
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // 2. 根据乒乓球颜色创建掩膜
    // 对于橙色乒乓球，HSV范围通常在 (0, 100, 100) 到 (20, 255, 255)
    // 如果是白色乒乓球，HSV范围通常在 (0, 0, 200) 到 (255, 20, 255)
    cv::Scalar lower_bound(0, 100, 100);
    cv::Scalar upper_bound(20, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv_image, lower_bound, upper_bound, mask);

    // 3. 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 4. 找到最大的轮廓（假设是乒乓球）
    double max_area = 0.0;
    std::vector<cv::Point> largest_contour;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > max_area) {
            max_area = area;
            largest_contour = contour;
        }
    }

    // 5. 如果找到的轮廓面积足够大，则计算其中心
    if (max_area > 50) { // 阈值可以根据实际情况调整
        cv::Moments m = cv::moments(largest_contour);
        if (m.m00 > 0) {
            ball_position.x = m.m10 / m.m00;
            ball_position.y = m.m01 / m.m00;
        }
    }

    return ball_position;
}

} // namespace kalman_trajectory_predictor
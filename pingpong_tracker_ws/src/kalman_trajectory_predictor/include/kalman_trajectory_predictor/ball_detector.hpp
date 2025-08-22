#ifndef KALMAN_TRAJECTORY_PREDICTOR__BALL_DETECTOR_HPP_
#define KALMAN_TRAJECTORY_PREDICTOR__BALL_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace kalman_trajectory_predictor {

class BallDetector {
public:
    BallDetector();

    // 核心方法：处理图像并返回乒乓球坐标
    cv::Point2f processImage(const cv::Mat& image);

private:
    // 如果有任何私有成员或辅助方法，可以在这里声明
};

} // namespace kalman_trajectory_predictor

#endif // KALMAN_TRAJECTORY_PREDICTOR__BALL_DETECTOR_HPP_
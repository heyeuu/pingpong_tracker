#pragma once

#include "opencv2/opencv.hpp"

class KalmanFilter {
public:
    KalmanFilter();

    // 预测下一时刻的状态
    cv::Point2f predict();

    // 使用测量值更新状态
    void update(const cv::Point2f& measurement);

    // 获取当前滤波后的位置
    cv::Point2f getFilteredPosition() const;

private:
    cv::KalmanFilter kf_;
};


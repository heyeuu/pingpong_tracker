#pragma once

#include "opencv2/opencv.hpp"

class KalmanFilter {
public:
  KalmanFilter() {
    int state_dim = 4;

    int measurement_dim = 2;

    int control_dim = 0;

    kf_.init(state_dim, measurement_dim, control_dim);

    kf_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0,
                            0, 1, 0, 0, 0, 0, 1);

    kf_.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);

    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-1));

    kf_.statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
  };

  cv::Point2f predict() {
    cv::Mat prediction = kf_.predict();
    return {prediction.at<float>(0, 0), prediction.at<float>(1, 0)};
  }

  void update(const cv::Point2f &measurement) {
    cv::Mat measurement_mat =
        (cv::Mat_<float>(2, 1) << measurement.x, measurement.y);
    kf_.correct(measurement_mat);
  }

private:
  cv::KalmanFilter kf_;
};

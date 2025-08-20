#pragma once

#include "opencv2/opencv.hpp"

class KalmanFilter {
public:
  KalmanFilter() {
    int state_dim = 6;

    int measurement_dim = 3;

    int control_dim = 0;

    kf_.init(state_dim, measurement_dim, control_dim);

    kf_.transitionMatrix =
        (cv::Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1,
         0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);

    kf_.measurementMatrix = (cv::Mat_<float>(3, 6) << 1, 0, 0, 0, 0, 0, 0, 1, 0,
                             0, 0, 0, 0, 0, 1, 0, 0, 0);

    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-2));

    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-1));

    kf_.statePost = (cv::Mat_<float>(6, 1) << 0, 0, 0, 0, 0, 0, 0);
  };

  cv::Point3f predict() {
    cv::Mat prediction = kf_.predict();

    return cv::Point3f(prediction.at<float>(0, 0), prediction.at<float>(1, 0),
                       prediction.at<float>(2, 0));
  }

  void update(const cv::Point3f &measurement) {
    cv::Mat measurement_mat =
        (cv::Mat_<float>(3, 1) << measurement.x, measurement.y, measurement.z);

    kf_.correct(measurement_mat);
  }

  cv::Point3f getFilteredPosition() const {
    return {kf_.statePost.at<float>(0, 0), kf_.statePost.at<float>(1, 0),
            kf_.statePost.at<float>(2, 0)};
  }

private:
  cv::KalmanFilter kf_;
};

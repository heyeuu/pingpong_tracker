#pragma once

#include "opencv2/opencv.hpp"

class KalmanFilter
{
public:
  KalmanFilter()
  {
    int state_dim = 6;

    int measurement_dim = 3;

    int control_dim = 0;

    kf_.init(state_dim, measurement_dim, control_dim);

    kf_.transitionMatrix =
      (cv::Mat_<double>(6, 6) << 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1,
       0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);

    kf_.measurementMatrix =
      (cv::Mat_<double>(3, 6) << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0);

    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-2));

    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-1));

    kf_.statePost = (cv::Mat_<double>(6, 1) << 0, 0, 0, 0, 0, 0, 0);
  };

  cv::Point3f predict()
  {
    cv::Mat prediction = kf_.predict();

    return cv::Point3d(
      prediction.at<double>(0, 0), prediction.at<double>(1, 0), prediction.at<double>(2, 0));
  }

  void update(const cv::Point3d & measurement)
  {
    cv::Mat measurement_mat =
      (cv::Mat_<double>(3, 1) << measurement.x, measurement.y, measurement.z);

    kf_.correct(measurement_mat);
  }

  cv::Point3d getFilteredPosition() const
  {
    return cv::Point3d(
      kf_.statePost.at<double>(0, 0), kf_.statePost.at<double>(1, 0),
      kf_.statePost.at<double>(2, 0));
  }

private:
  cv::KalmanFilter kf_;
};

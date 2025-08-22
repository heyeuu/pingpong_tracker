#include "kalman_trajectory_predictor/kalman_filter.hpp"

namespace kalman_trajectory_predictor {
KalmanFilter::KalmanFilter() {}
void KalmanFilter::update(const Eigen::VectorXd& z) {}
void KalmanFilter::predict() {}
Eigen::VectorXd KalmanFilter::getState() const { return Eigen::VectorXd(); }
} // namespace kalman_trajectory_predictor
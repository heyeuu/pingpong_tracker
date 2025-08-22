#ifndef KALMAN_TRAJECTORY_PREDICTOR__KALMAN_FILTER_HPP_
#define KALMAN_TRAJECTORY_PREDICTOR__KALMAN_FILTER_HPP_

#include <eigen3/Eigen/Dense>

namespace kalman_trajectory_predictor
{

class KalmanFilter
{
public:
  KalmanFilter();

  // 核心方法：根据新的测量值更新状态
  void update(const Eigen::VectorXd& z);

  // 核心方法：预测下一时刻的状态
  void predict();

  // 获取当前状态
  Eigen::VectorXd getState() const;

private:
  // 这里可以定义你的状态转移矩阵、协方差矩阵等
  Eigen::MatrixXd A_; 
  Eigen::MatrixXd H_; 
  Eigen::MatrixXd Q_; 
  Eigen::MatrixXd R_; 
  Eigen::MatrixXd P_; 
  Eigen::VectorXd x_; 
};

} // namespace kalman_trajectory_predictor

#endif // KALMAN_TRAJECTORY_PREDICTOR__KALMAN_FILTER_HPP_
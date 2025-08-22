#ifndef KALMAN_TRAJECTORY_PREDICTOR__PNP_SOLVER_HPP_
#define KALMAN_TRAJECTORY_PREDICTOR__PNP_SOLVER_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace kalman_trajectory_predictor {

class PnpSolver {
public:
    PnpSolver();

    // 核心方法：求解PnP，返回相机姿态（旋转向量和平移向量）
    bool solvePnP(
        const std::vector<cv::Point3f>& object_points, const std::vector<cv::Point2f>& image_points,
        cv::Mat& rvec, cv::Mat& tvec);

private:
    // 你可能需要相机内参矩阵、畸变系数等
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

} // namespace kalman_trajectory_predictor

#endif // KALMAN_TRAJECTORY_PREDICTOR__PNP_SOLVER_HPP_
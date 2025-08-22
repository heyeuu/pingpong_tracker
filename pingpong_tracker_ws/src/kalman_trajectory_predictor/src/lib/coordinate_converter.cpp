#ifndef KALMAN_TRAJECTORY_PREDICTOR__COORDINATE_CONVERTER_HPP_
#define KALMAN_TRAJECTORY_PREDICTOR__COORDINATE_CONVERTER_HPP_

#include <opencv2/opencv.hpp>

namespace kalman_trajectory_predictor
{

class CoordinateConverter
{
public:
  CoordinateConverter();

  // 核心方法：将2D像素坐标转换为3D世界坐标
  cv::Point3f pixelToWorld(const cv::Point2f& pixel_point);

private:
  // 你可能需要相机内参矩阵、畸变系数等
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};

} // namespace kalman_trajectory_predictor

#endif // KALMAN_TRAJECTORY_PREDICTOR__COORDINATE_CONVERTER_HPP_
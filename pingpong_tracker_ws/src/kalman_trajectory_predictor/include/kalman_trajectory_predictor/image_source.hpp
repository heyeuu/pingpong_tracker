#ifndef KALMAN_TRAJECTORY_PREDICTOR__IMAGE_SOURCE_HPP_
#define KALMAN_TRAJECTORY_PREDICTOR__IMAGE_SOURCE_HPP_

#include <opencv2/opencv.hpp>

namespace kalman_trajectory_predictor {

class ImageSource {
public:
    virtual bool getNextImage(cv::Mat& image) = 0;
    virtual ~ImageSource() = default;
};

class CameraImageSource : public ImageSource {
public:
    explicit CameraImageSource(int camera_id = 0);
    bool getNextImage(cv::Mat& image) override;

private:
    cv::VideoCapture cap_;
};

class VideoFileImageSource : public ImageSource {
public:
    explicit VideoFileImageSource(const std::string& file_path);
    bool getNextImage(cv::Mat& image) override;

private:
    cv::VideoCapture cap_;
};

} // namespace kalman_trajectory_predictor

#endif // KALMAN_TRAJECTORY_PREDICTOR__IMAGE_SOURCE_HPP_
#include "kalman_trajectory_predictor/image_source.hpp"

namespace kalman_trajectory_predictor {

CameraImageSource::CameraImageSource(int camera_id) { cap_.open(camera_id); }
bool CameraImageSource::getNextImage(cv::Mat& image) {
    if (!cap_.isOpened()) {
        return false;
    }
    cap_ >> image;
    return !image.empty();
}

VideoFileImageSource::VideoFileImageSource(const std::string& file_path) { cap_.open(file_path); }
bool VideoFileImageSource::getNextImage(cv::Mat& image) {
    if (!cap_.isOpened()) {
        return false;
    }
    cap_ >> image;
    return !image.empty();
}
} // namespace kalman_trajectory_predictor
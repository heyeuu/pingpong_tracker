#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "kalman_trajectory_predictor/image_source.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

namespace kalman_trajectory_predictor {

class ImagePublisherNode : public rclcpp::Node {
public:
    explicit ImagePublisherNode(const rclcpp::NodeOptions& options)
        : Node("image_publisher_node", options) {

        // Declare parameters once at the beginning of the constructor.
        this->declare_parameter("image_source", "video");
        this->declare_parameter("video_path", "");
        this->declare_parameter("camera_info_url", "");
        this->declare_parameter("camera_id", 0);

        // Get the parameter values into local variables. This should also only be done once.
        // auto source_type = this->get_parameter("image_source").get_value<std::string>();
        // std::string video_path = this->get_parameter("video_path").get_value<std::string>();
        // std::string cam_info_url =
        // this->get_parameter("camera_info_url").get_value<std::string>();

        std::string source_type = "video";
        std::string video_path =
            "src/kalman_trajectory_predictor/test_video/pingpong_video.mp4"; // 修改为你的绝对或相对路径
        std::string cam_info_url =
            "file:///workspaces/pingpong_tracker/pingpong_tracker_ws/src/"
            "kalman_trajectory_predictor/config/camera_info.yaml"; // 修改为你的绝对或相对路径

        // RCLCPP_INFO(this->get_logger(), "DEBUG: Got image_source: %s", source_type.c_str());
        // RCLCPP_INFO(this->get_logger(), "DEBUG: Got video_path: %s", video_path.c_str());
        // RCLCPP_INFO(
        //     this->get_logger(), "DEBUG: Got camera_info_url: %s",
        //     cam_info_url.c_str());                               

        if (source_type == "camera") {
            RCLCPP_INFO(this->get_logger(), "Using camera as image source.");
            auto camera_id = this->get_parameter("camera_id").get_value<int>();
            image_source_ = std::make_unique<CameraImageSource>(camera_id);
        } else if (source_type == "video") {
            RCLCPP_INFO(this->get_logger(), "Using video file as image source.");
            if (video_path.empty()) {
                RCLCPP_FATAL(this->get_logger(), "Video path parameter is not set!");
                rclcpp::shutdown();
                return;
            }
            image_source_ = std::make_unique<VideoFileImageSource>(video_path);
        } else {
            RCLCPP_FATAL(this->get_logger(), "Invalid image source type: %s", source_type.c_str());
            rclcpp::shutdown();
            return;
        }

        if (!image_source_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create image source.");
            rclcpp::shutdown();
            return;
        }

        camera_pub_ = image_transport::create_camera_publisher(this, "camera/image_raw");

        auto cam_info_manager =
            std::make_unique<camera_info_manager::CameraInfoManager>(this, "mv_camera");

        if (cam_info_manager->validateURL(cam_info_url)) {
            cam_info_manager->loadCameraInfo(cam_info_url);
            camera_info_msg_ = cam_info_manager->getCameraInfo();
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", cam_info_url.c_str());
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / 120),
            std::bind(&ImagePublisherNode::publishImage, this));
    }

private:
    std::unique_ptr<ImageSource> image_source_;
    image_transport::CameraPublisher camera_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    void publishImage() {
        cv::Mat frame;
        if (!image_source_->getNextImage(frame)) {
            RCLCPP_INFO(this->get_logger(), "End of stream, shutting down.");
            rclcpp::shutdown();
            return;
        }

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Empty frame from image source.");
            return;
        }

        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = "camera_optical_frame";

        camera_info_msg_.header.stamp = image_msg->header.stamp;
        camera_info_msg_.header.frame_id = image_msg->header.frame_id;

        camera_pub_.publish(*image_msg, camera_info_msg_);
    }
};

} // namespace kalman_trajectory_predictor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kalman_trajectory_predictor::ImagePublisherNode)
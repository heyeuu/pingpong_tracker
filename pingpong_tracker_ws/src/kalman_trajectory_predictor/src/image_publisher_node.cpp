#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImagePublisherNode : public rclcpp::Node {
public:
  ImagePublisherNode() : Node("image_publisher_node") {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     [this] { publish_image(); });

    // 读取图像文件
    std::string image_path =
        "/workspaces/pingpong_tracker/docs/images/test_ball.jpg";
    image_ = cv::imread(image_path, cv::IMREAD_COLOR);

    if (image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Could not read the image from path: %s",
                   image_path.c_str());
    }
  }

private:
  void publish_image() {
    if (!image_.empty()) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_)
                     .toImageMsg();
      publisher_->publish(*msg);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat image_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
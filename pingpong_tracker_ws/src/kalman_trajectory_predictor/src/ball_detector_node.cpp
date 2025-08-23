#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include "kalman_trajectory_predictor/ball_detector.hpp"

namespace kalman_trajectory_predictor {

class BallDetectorNode : public rclcpp::Node {
public:
    explicit BallDetectorNode(const rclcpp::NodeOptions& options)
        : Node("ball_detector_node", options) {
        // 创建图像订阅器
        // 订阅话题名为 "camera/image_raw"
        // 回调函数为 this->imageCallback
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&BallDetectorNode::imageCallback, this, std::placeholders::_1));

        // 创建点坐标发布器
        // 发布话题名为 "pingpong/ball_position"
        publisher_ =
            this->create_publisher<geometry_msgs::msg::Point>("pingpong/ball_position", 10);

        // 实例化我们自己的BallDetector模块
        ball_detector_ = std::make_unique<BallDetector>();
    }

private:
    // 订阅回调函数，每当收到图像消息时被调用
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 使用cv_bridge将ROS2图像消息转换为OpenCV的Mat类型
            cv_image_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");

            // 调用我们封装好的图像处理模块来获取乒乓球坐标
            cv::Point2f ball_position = ball_detector_->processImage(cv_image_ptr_->image);

            // 检查是否成功检测到乒乓球
            // 如果坐标是-1，说明没有找到
            if (ball_position.x >= 0 && ball_position.y >= 0) {
                auto point_msg = geometry_msgs::msg::Point();
                point_msg.x = static_cast<double>(ball_position.x);
                point_msg.y = static_cast<double>(ball_position.y);
                point_msg.z = 0.0; // 2D坐标，z设为0

                // 发布消息
                publisher_->publish(point_msg);
                RCLCPP_INFO(
                    this->get_logger(), "Published ball position: (%.2f, %.2f)", point_msg.x,
                    point_msg.y);
            } else {
                RCLCPP_INFO(this->get_logger(), "No ball detected.");
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

    std::unique_ptr<BallDetector> ball_detector_;

    cv_bridge::CvImagePtr cv_image_ptr_;
};

} // namespace kalman_trajectory_predictor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kalman_trajectory_predictor::BallDetectorNode)
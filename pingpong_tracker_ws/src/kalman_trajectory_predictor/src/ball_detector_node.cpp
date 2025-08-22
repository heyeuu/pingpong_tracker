// 引入ROS2核心库
#include <rclcpp/rclcpp.hpp>

// 引入ROS2标准消息类型
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>

// 引入图像和ROS消息转换工具
#include <cv_bridge/cv_bridge.h>

// 引入我们自己的核心功能模块头文件
// 注意：这里已经修正为正确的ball_detector.hpp
#include "kalman_trajectory_predictor/ball_detector.hpp"

// 使用一个命名空间来组织代码，符合现代C++和ROS2规范
namespace kalman_trajectory_predictor {

class BallDetectorNode : public rclcpp::Node {
public:
    // 构造函数
    explicit BallDetectorNode()
        : Node("ball_detector_node") {
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
                // 封装ROS2消息
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

    // 声明ROS2订阅器和发布器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

    // 使用智能指针来管理BallDetector模块实例
    std::unique_ptr<BallDetector> ball_detector_;

    // cv_bridge指针
    cv_bridge::CvImagePtr cv_image_ptr_;
};

} // namespace kalman_trajectory_predictor

// 主函数
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // 使用rclcpp::spin来运行节点
    // std::make_shared确保节点对象被正确地管理
    rclcpp::spin(std::make_shared<kalman_trajectory_predictor::BallDetectorNode>());

    rclcpp::shutdown();
    return 0;
}
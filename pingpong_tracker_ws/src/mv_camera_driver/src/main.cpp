#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("image_publisher_node");

  auto publisher =
      node->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);

  sensor_msgs::msg::Image image_msg;
  image_msg.header.frame_id = "camera_frame";
  image_msg.width = 640;
  image_msg.height = 480;
  image_msg.encoding = "bgr8";
  image_msg.is_bigendian = false;
  image_msg.step = image_msg.width * 3;
  image_msg.data.resize(image_msg.step * image_msg.height, 0);

  rclcpp::WallRate loop_rate(1.0); // 1 Hz

  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "Publishing image message");
    publisher->publish(image_msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

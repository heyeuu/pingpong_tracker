#include "rclcpp_components/register_node_macro.hpp"
#include <rclcpp/rclcpp.hpp>

namespace kalman_trajectory_predictor {

class ImageVisualizerNode : public rclcpp::Node {
public:
    explicit ImageVisualizerNode(const rclcpp::NodeOptions& options)
        : Node("image_visualizer_node", options) {
        // 这里是空的
    }
};

} // namespace kalman_trajectory_predictor

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_trajectory_predictor::ImageVisualizerNode)
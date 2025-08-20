#include "rclcpp/rclcpp.hpp"

#include "kalman_trajectory_predictor/kalman_filter.hpp"
#include "kalman_trajectory_predictor/predictor_node.hpp"

class PredictorNode;
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PredictorNode>());
  rclcpp::shutdown();
  return 0;
}

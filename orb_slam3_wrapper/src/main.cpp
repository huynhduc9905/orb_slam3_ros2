#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "orb_slam3_wrapper/wrapper_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orb_slam3_wrapper::WrapperNode>());
  rclcpp::shutdown();
  return 0;
}

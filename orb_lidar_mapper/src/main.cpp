#include <rclcpp/rclcpp.hpp>
#include "orb_lidar_mapper/mapper_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orb_lidar_mapper::MapperNode>());
  rclcpp::shutdown();
  return 0;
}

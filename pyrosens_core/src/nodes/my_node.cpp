// filename: src/nodes/my_node.cpp (note *_node.cpp)
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("my_node");
  RCLCPP_INFO(node->get_logger(), "my_node started");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

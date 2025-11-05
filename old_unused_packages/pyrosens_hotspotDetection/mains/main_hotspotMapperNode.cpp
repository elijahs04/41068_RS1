#include <rclcpp/rclcpp.hpp>
#include "hotspotDetection_pyrosens/hotspotMapperNode.hpp"  // your class header

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HotspotMapperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

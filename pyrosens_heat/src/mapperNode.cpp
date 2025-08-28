#include "rclcpp/rclcpp.hpp"
class MapperNode : public rclcpp::Node {
public:
  MapperNode() : rclcpp::Node("mapperNode") {}
};
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapperNode>());
  rclcpp::shutdown();
  return 0;
}
#include "pyrosens_heat/mapperNode.hpp"
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapperNode>());
  rclcpp::shutdown();
  return 0;
}
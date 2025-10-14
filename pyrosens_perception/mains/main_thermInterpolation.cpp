#include "thermal_pyrosens/thermInterpolationNode.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermInterpolationNode>());
  rclcpp::shutdown();
  return 0;
}
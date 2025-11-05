#include "realSens_pyrosens/wind_InterpolationNode.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WindInterpolationNode>());
  rclcpp::shutdown();
  return 0;
}
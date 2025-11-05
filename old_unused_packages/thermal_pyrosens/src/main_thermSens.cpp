#include "thermal_pyrosens/thermSensNode.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermSensNode>());
  rclcpp::shutdown();
  return 0;
}

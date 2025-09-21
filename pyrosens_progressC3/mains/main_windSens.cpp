#include "wind_pyrosens/windSensNode.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WindSensNode>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "thermal_pyrosens/thermalSimNode.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalSimNode>());
  rclcpp::shutdown();
  return 0;
}

#include "wind_pyrosens/sensorNode.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorNode>());
  rclcpp::shutdown();
  return 0;
}

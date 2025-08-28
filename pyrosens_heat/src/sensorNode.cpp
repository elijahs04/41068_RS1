#include "rclcpp/rclcpp.hpp"
class SensorNode : public rclcpp::Node {
public:
  SensorNode() : rclcpp::Node("sensorNode") {}
};
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorNode>());
  rclcpp::shutdown();
  return 0;
}
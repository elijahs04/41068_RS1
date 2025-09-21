#include "rclcpp/rclcpp.hpp"
#include "thermal_pyrosens/thermInterpolationNode.hpp"
#include "thermal_pyrosens/thermSensNode.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto therm_interp = std::make_shared<ThermInterpolationNode>();
  auto therm_sensor = std::make_shared<ThermSensorNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(therm_interp);
  exec.add_node(therm_sensor);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}

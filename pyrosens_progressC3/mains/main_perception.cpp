#include "rclcpp/rclcpp.hpp"
#include "thermal_pyrosens/thermInterpolationNode.hpp"
#include "thermal_pyrosens/thermSensNode.hpp"
#include "wind_pyrosens/windInterpolationNode.hpp"
#include "wind_pyrosens/windSensNode.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto therm_interp = std::make_shared<ThermInterpolationNode>();
  auto therm_sensor = std::make_shared<ThermSensNode>();
  auto wind_interp  = std::make_shared<WindInterpolationNode>();
  auto wind_sensor  = std::make_shared<WindSensNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(therm_interp);
  exec.add_node(therm_sensor);
  exec.add_node(wind_interp);
  exec.add_node(wind_sensor);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}

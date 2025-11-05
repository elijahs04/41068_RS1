#include "rclcpp/rclcpp.hpp"
#include "thermal_pyrosens/thermInterpolationNode.hpp"
#include "wind_pyrosens/windInterpolationNode.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto therm = std::make_shared<ThermInterpolationNode>();
  auto wind  = std::make_shared<WindInterpolationNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(therm);
  exec.add_node(wind);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

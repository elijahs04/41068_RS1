#include "rclcpp/rclcpp.hpp"
#include "thermal_pyrosens/thermalSimNode.hpp"
#include "wind_pyrosens/windSimNode.hpp"

int main(int argc, char **argv) {
  // Init (intra-process optional; sims don't cross-talk, so default is fine)
  rclcpp::init(argc, argv);

  auto therm_sim = std::make_shared<ThermalSimNode>();
  auto wind_sim  = std::make_shared<WindSimNode>();

  // Spin both simulators in one process
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(therm_sim);
  exec.add_node(wind_sim);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
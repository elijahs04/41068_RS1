#include "rclcpp/rclcpp.hpp"
#include "pyrosens_prediction/predictionNode.hpp"
#include "pyrosens_prediction/predictionSim.hpp"

#include <string>
#include <vector>

int main(int argc, char * argv[])
{
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  bool enable_sim = false;
  for (const auto & arg : args) {
    if (arg == "--with-sim" || arg == "--enable-sim") {
      enable_sim = true;
    } else if (arg == "--no-sim" || arg == "--live") {
      enable_sim = false;
    }
  }

  rclcpp::init(argc, argv);
  auto predict = std::make_shared<PredictionNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(predict);

  std::shared_ptr<PredictionSim> sim;
  if (enable_sim) {
    sim = std::make_shared<PredictionSim>();
    exec.add_node(sim);
    RCLCPP_WARN(predict->get_logger(), "PredictionSim enabled; publishing synthetic data.");
  } else {
    RCLCPP_INFO(predict->get_logger(), "PredictionSim disabled; expecting pyrosens_progress topics.");
  }

  exec.spin();

  rclcpp::shutdown();
  return 0;
}

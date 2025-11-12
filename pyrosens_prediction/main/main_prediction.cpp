/*
this main needs to spin 1 or both nodes:
node 1 is the prediction node 
we can spin separately to test something, if there's a pain point
the prediction node is the logic of the data it is fed (wind, heat, point stamps)
the wind is running (vector), heat is (here), so it should go here to here
basic flow: it will move from point in direction 
advanced flow: it will move from point to point

the main spins the node, which is the header file 
we're splitting them up for good coding practice 
the header runs the constructor, which has functions within it
the logic is in the header, functions, constructor, subscribers, publishes, service calls, 

identify publishers and subscribers
identify functions i may need 

simulation node that is going to pass a point cloud of points that have heat and wind, defined in the simulation node
this will publish to x topic and the prediction node will subscribe to that topic
the prediction node then interpolates this data (will need sexy functions)
the prediction node will then publish to the terminal
need a timer to call the function every x seconds 

sim node contains all the data, it is how we validate that the prediction model works 
*/

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

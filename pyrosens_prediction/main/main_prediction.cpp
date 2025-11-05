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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto predict = std::make_shared<PredictionNode>();
  auto sim = std::make_shared<PredictionSim>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(predict);
  exec.add_node(sim);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
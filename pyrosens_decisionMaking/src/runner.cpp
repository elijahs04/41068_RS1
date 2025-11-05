#include <rclcpp/rclcpp.hpp>
#include "decisionMaking.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionMaking>();




    // create an executor 
    rclcpp::executors::MultiThreadedExecutor executor;

    // add the node to the executor
    executor.add_node(node);

    // spin the executor
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
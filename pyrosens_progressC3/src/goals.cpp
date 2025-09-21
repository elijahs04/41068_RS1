#include "goals.h"

Goals::Goals() : rclcpp::Node("goals_execution") {
    RCLCPP_INFO(this->get_logger(), "Goals action client started");

    // Create action client
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Wait for the action server
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available!");
        return;
    }

    
    runGoals();
}

Goals::~Goals() {
    if (runThread.joinable()) {
        runThread.join();
    }
}

void Goals::runGoals()
{
    std::vector<std::tuple<double,double,double>> goals = {
        {7.0, -5.0, 0.0},
        {-0.5, -7.5, 0.0},
        {-2.0, -2.5, 0.0}
    };

    for (auto & g : goals) {
        double x, y, theta;
        std::tie(x, y, theta) = g;

        if (!sendGoalAndWait(x, y, theta)) {
            RCLCPP_WARN(this->get_logger(), "Stopping goal sequence due to failure");
            break;
        }
    }
}


bool Goals::sendGoalAndWait(double x, double y, double theta)
{
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal to [%.2f, %.2f]", x, y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // Send goal asynchronously
    auto goal_handle_future = client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for goal to be accepted
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
        return false;
    }

    // Wait for the result
    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get goal result");
        return false;
    }

    auto result = result_future.get();
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
            return true;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Goal was aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled.");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
    }

    return false;
}
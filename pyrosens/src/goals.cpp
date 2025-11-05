#include "goals.h"


Goals::Goals() : rclcpp::Node("goals_execution") {
    RCLCPP_INFO(this->get_logger(), "Goals action client started");

    // Create action client
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
    //Publisher to cmd_vel 
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    //subscribe to plan 
    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&Goals::planCallback, this, std::placeholders::_1));

    // Subscribe to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10, std::bind(&Goals::odomCallback, this, std::placeholders::_1));


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

void Goals::planCallback(const nav_msgs::msg::Path::SharedPtr msg){
    if(msg->poses.size() <2) return; 

    double totalDistance = 0.0;
    for (size_t i = 1; i < msg->poses.size(); i++){
        const auto &p1 = msg->poses[i-1].pose.position;
        const auto &p2 = msg->poses[i].pose.position;
        totalDistance += std::hypot(p2.x - p1.x, p2.y - p1.y);   
    }

    currentPathLength_ = totalDistance;    

}

void Goals::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    currentPose_ = msg->pose.pose;
}

void Goals::runGoals()
{
    std::vector<std::tuple<double,double,double,double,double>> goals = {
        {1.56, 1.26, 0.30, 0.95, 0.0},
        {-3, -3, 0.30, 0.95, 0.0},
        {9, 8, 0.45, 0.89, 0.0}
    };

    for (auto & g : goals) {
        double x, y, qz, qw, theta;
        std::tie(x, y, qz, qw, theta) = g;

        if (!sendGoalAndWait(x, y, qz, qw, theta)) {
            RCLCPP_WARN(this->get_logger(), "Goal Not reached moving upward");
            moveUpward(5);
            sendGoalAndWait(x, y, qz, qw, theta);
            break;
        }

        moveUpward(10);
        clearCostmap();
    }
}


bool Goals::sendGoalAndWait(double x, double y, double qz, double qw,double theta)
{
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.z = qz;
    goal_msg.pose.pose.orientation.w = qw;

    RCLCPP_INFO(this->get_logger(), "Sending goal to [%.2f, %.2f]", x, y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // Send goal asynchronously
    auto goal_handle_future = client_->async_send_goal(goal_msg, send_goal_options);

    currentGoalPose_ = goal_msg.pose;

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

    rclcpp::sleep_for(std::chrono::seconds(2));

    // Check path efficiency
    if (!checkGoalReachable()) {
        RCLCPP_WARN(this->get_logger(), "Goal path seems inefficient or blocked — canceling and retrying later");
        client_->async_cancel_all_goals();

        moveUpward(5.0); // moves drone up 5 meters 
        clearCostmap();

        RCLCPP_INFO(this->get_logger(), "Retrying goal from higher altitude...");
        return sendGoalAndWait(x, y, qz, qw, theta);
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

void Goals::moveUpward(double meters){
    geometry_msgs::msg::Twist cmd;
    cmd.linear.z =0.5;

    double duration = meters /cmd.linear.z;
    auto start = this->now();

    rclcpp::Rate rate(10);
    while ((this->now() - start).seconds() < duration && rclcpp::ok()) {
        vel_pub_->publish(cmd);
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep(); 
    }

    geometry_msgs::msg::Twist stop_cmd;
    vel_pub_->publish(stop_cmd);
    rclcpp::sleep_for(std::chrono::seconds(2));


}

bool Goals::checkGoalReachable(){

    double dx = currentGoalPose_.pose.position.x - currentPose_.position.x;
    double dy = currentGoalPose_.pose.position.y - currentPose_.position.y;

    double directDistance = std::hypot(dx,dy);
    if (directDistance < 0.05) {
        RCLCPP_INFO(this->get_logger(), "Already at Goal");
        return true; //already at goal 
    }

    if (currentPathLength_<0.0) {
        RCLCPP_INFO(this->get_logger(), "Already at Goal");
        return true; // no plan yet
    }

    double ratio = currentPathLength_/ directDistance; 

    RCLCPP_INFO(this->get_logger(), 
        "Direct dist = %.2f, Path len - %.2f, Ratio = %.2f", directDistance, currentPathLength_, ratio);

    if (ratio >2.0) { 
        RCLCPP_WARN(this->get_logger(), "Path too long compared to direct distance! (%.2fx)", ratio);
        return false;
    }

    return true; 

}

void Goals::clearCostmap() {
    std::vector<std::string> services = {
        "/global_costmap/clear_entirely_global_costmap",
        "/local_costmap/clear_entirely_local_costmap"
    };

    for (const auto &srv_name : services) {
        auto client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(srv_name);

        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "%s service not available!", srv_name.c_str());
            continue;
        }

        
        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Cleared costmap via %s", srv_name.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Timeout while calling %s", srv_name.c_str());
        }
    }
}


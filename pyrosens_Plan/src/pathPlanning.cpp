#include "pathPlanning.h"


PathPlanning::PathPlanning() : rclcpp::Node("goals_execution") {
    RCLCPP_INFO(this->get_logger(), "Goals action client started");

    // Create action client
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
    //Publisher to cmd_vel 
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    //subscribe to plan 
    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&PathPlanning::planCallback, this, std::placeholders::_1));

    // Subscribe to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PathPlanning::odomCallback, this, std::placeholders::_1));


    // Wait for the action server
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available!");
        return;
    }

    
}

PathPlanning::~PathPlanning() {
    if (runThread.joinable()) {
        runThread.join();
    }
}

void PathPlanning::planCallback(const nav_msgs::msg::Path::SharedPtr msg){
    if(msg->poses.size() <2) return; 

    double totalDistance = 0.0;
    for (size_t i = 1; i < msg->poses.size(); i++){
        const auto &p1 = msg->poses[i-1].pose.position;
        const auto &p2 = msg->poses[i].pose.position;
        totalDistance += std::hypot(p2.x - p1.x, p2.y - p1.y);   
    }

    currentPathLength_ = totalDistance;    

}

void PathPlanning::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    currentPose_ = msg->pose.pose;
}






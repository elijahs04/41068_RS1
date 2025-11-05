#include "pathPlanning.h"


PathPlanning::PathPlanning() : rclcpp::Node("planning_execution") {
    RCLCPP_INFO(this->get_logger(), "Planning action client started");

    // Create action client
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
    
    // Subscribe to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PathPlanning::odomCallback, this, std::placeholders::_1));

    pointCloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/hotspot/point_cloud", 10, std::bind(&PathPlanning::pointCloudCallback, this, std::placeholders::_1));

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

void PathPlanning::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg){
    std::lock_guard<std::mutex> lock(dataMutex_);
    // Container for all XY points

        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 with %u points", msg->width * msg->height);
        
        xyPoints_.reserve(msg->width * msg->height);

        // Iterate through the PointCloud2 fields natively
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
                                                         iter_y(*msg, "y");
             iter_x != iter_x.end();
             ++iter_x, ++iter_y)
        {
            float x = *iter_x;
            float y = *iter_y;

            // Skip NaNs or invalid points
            if (!std::isfinite(x) || !std::isfinite(y))
                continue;

            xyPoints_.emplace_back(x, y);

            RCLCPP_INFO(this->get_logger(), "xyPoints (%zu):", xyPoints_.size());
            for (const auto &p : xyPoints_)
                RCLCPP_INFO(this->get_logger(), "(%.3f, %.3f)", p.first, p.second);
        }

}

void PathPlanning::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    std::lock_guard<std::mutex> lock(dataMutex_);
    currentPose_ = msg->pose.pose;
}

void PathPlanning::findNextGoal(){
    sensor_msgs::msg::PointCloud2 pointCloud;
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        auto xyPoints = xyPoints_;
    }

}

void PathPlanning::sendNextGoal(){

}

#include "decisionMaking.h"
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <limits>

DecisionMaking::DecisionMaking() : rclcpp::Node("planning_execution") {
    RCLCPP_INFO(this->get_logger(), "Planning action client started");

    // Create action client
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
    
    // Subscribe to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10, std::bind(&DecisionMaking::odomCallback, this, std::placeholders::_1));

    pointCloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/hotspots/points_cloud",
        rclcpp::SensorDataQoS(),
        std::bind(&DecisionMaking::pointCloudCallback, this, std::placeholders::_1));
    
    goalsPublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/next_goal", 10);
    
    

    // Wait for the action server
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available!");
        return;
    }

    
}

DecisionMaking::~DecisionMaking() {
    if (runThread.joinable()) {
        runThread.join();
    }
}

void DecisionMaking::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
    {std::lock_guard<std::mutex> lock(dataMutex_);

    RCLCPP_DEBUG(this->get_logger(), "Received PointCloud2 with %u points", msg->width * msg->height);

    xyPoints_.clear();
    xyPoints_.reserve(msg->width * msg->height);

    float min_x = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
                                                     iter_y(*msg, "y");
         iter_x != iter_x.end();
         ++iter_x, ++iter_y)
    {
        const float x = *iter_x;
        const float y = *iter_y;

        if (!std::isfinite(x) || !std::isfinite(y)) {
            continue;
        }

        xyPoints_.emplace_back(x, y);
    }

    if (xyPoints_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Point cloud contained no valid XY points");
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Stored %zu points;",
        xyPoints_.size());
    }
}

void DecisionMaking::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    std::lock_guard<std::mutex> lock(dataMutex_);
    currentPose_ = msg->pose.pose;
}

void DecisionMaking::findFirstGoal(){
    geometry_msgs::msg::Pose current_pose;
    std::vector<std::pair<float, float>> points;
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        current_pose = currentPose_;
        points = xyPoints_;
    }

    if (points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot compute next goal: no cached points");
        return;
    }

    const double current_x = current_pose.position.x;
    const double current_y = current_pose.position.y;

    double min_distance_sq = std::numeric_limits<double>::infinity();
    std::pair<float, float> closest_point{0.0F, 0.0F};

    for (const auto &point : points) {
        const double dx = static_cast<double>(point.first) - current_x;
        const double dy = static_cast<double>(point.second) - current_y;
        const double distance_sq = dx * dx + dy * dy;

        if (distance_sq < min_distance_sq) {
            min_distance_sq = distance_sq;
            closest_point = point;
        }
    }

    const double distance = std::sqrt(min_distance_sq);
    if (distance < 1e-6) {
        RCLCPP_WARN(this->get_logger(), "Closest point coincides with robot pose; skipping goal placement");
        return;
    }

    const double direction_x = (static_cast<double>(closest_point.first) - current_x) / distance;
    const double direction_y = (static_cast<double>(closest_point.second) - current_y) / distance;

    const double goal_x = static_cast<double>(closest_point.first) - direction_x * 3.0;
    const double goal_y = static_cast<double>(closest_point.second) - direction_y * 3.0;

    const double yaw = std::atan2(direction_y, direction_x);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    currentGoalPose_.header.stamp = this->now();
    currentGoalPose_.header.frame_id = "map";
    currentGoalPose_.pose.position.x = goal_x;
    currentGoalPose_.pose.position.y = goal_y;
    currentGoalPose_.pose.position.z = 0.0;
    currentGoalPose_.pose.orientation.x = q.x();
    currentGoalPose_.pose.orientation.y = q.y();
    currentGoalPose_.pose.orientation.z = q.z();
    currentGoalPose_.pose.orientation.w = q.w();

    RCLCPP_INFO(
        this->get_logger(),
        "Next goal placed 3.0 m from closest point (%.3f, %.3f) at (%.3f, %.3f) facing the point",
        static_cast<double>(closest_point.first),
        static_cast<double>(closest_point.second),
        goal_x,
        goal_y);
}

void DecisionMaking::sendNextGoal(){

}

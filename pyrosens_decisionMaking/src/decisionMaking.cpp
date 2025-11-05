#include "decisionMaking.h"
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>

DecisionMaking::DecisionMaking() : rclcpp::Node("planning_execution") {
    RCLCPP_INFO(this->get_logger(), "Planning action client started");

    // Create action client
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "/mission/navigate_to_pose");
    
    
    // Subscribe to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10, std::bind(&DecisionMaking::odomCallback, this, std::placeholders::_1));

    pointCloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/hotspots/points_cloud",
        rclcpp::SensorDataQoS(),
        std::bind(&DecisionMaking::pointCloudCallback, this, std::placeholders::_1));
    
    // Wait for the action server
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available!");
        return;
    }

    findNextFire();

    
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

    if (isCloseEnough(currentPose_, currentGoalPose_)){
        findNextFire();
    }
}

void DecisionMaking::findNextFire(){
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

    auto dense_point = findDenseCluster(points);

    const double current_x = current_pose.position.x;
    const double current_y = current_pose.position.y;

    double min_distance_sq = std::numeric_limits<double>::infinity();
    std::pair<float, float> closest_point{0.0F, 0.0F};

    for (const auto &point : points) {
        const double dx = static_cast<double>(point.first) - current_x;
        const double dy = static_cast<double>(point.second) - current_y;
        const double distance_sq = dx * dx + dy * dy;

        if ((distance_sq < min_distance_sq) && outsideFireZones(point.first, point.second) && closeToDensePoint(point.first, point.second, dense_point.first, dense_point.second)) {
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

    int numberFires = Fires_.size();

    Fires_.push_back({numberFires + 1, static_cast<int>(goal_x), static_cast<int>(goal_y), 2});

    RCLCPP_INFO(
        this->get_logger(),
        "Next goal placed 3.0 m from closest point (%.3f, %.3f) at (%.3f, %.3f) facing the point",
        static_cast<double>(closest_point.first),
        static_cast<double>(closest_point.second),
        goal_x,
        goal_y);
}



bool DecisionMaking::outsideFireZones(float x, float y){
    for(const auto& fire : Fires_){

        float fire_x = static_cast<float>(fire[1]);
        float fire_y = static_cast<float>(fire[2]);
        float fire_radius = static_cast<float>(fire[3]);

        if(x >= fire_x - fire_radius && x <= fire_x + fire_radius &&
           y >= fire_y - fire_radius && y <= fire_y + fire_radius){
               return false;
           }
    }
    return true;
}

bool DecisionMaking::closeToDensePoint(float x, float y, float dense_x, float dense_y){
    float threshold = 2; // 2 meters
    float dx = x - dense_x;
    float dy = y - dense_y;
    float distance_sq = dx * dx + dy * dy;
    return distance_sq <= threshold;
}

std::pair<float, float> DecisionMaking::findDenseCluster( const std::vector<std::pair<float, float>> points) {
    float radius = 1.0;
    int clusterSize = 10;

    for (size_t i = 0; i < points.size(); ++i) {
        int count = 0;
        const auto& p1 = points[i];

        for (size_t j = 0; j < points.size(); ++j) {
            if (i == j) continue;
            const auto& p2 = points[j];
            float dx = p1.first  - p2.first;
            float dy = p1.second - p2.second;
            float dist = std::sqrt(dx * dx + dy * dy);

            if (dist <= radius)
                count++;
        }

        if (count >= clusterSize)
            return p1;  // found a dense cluster
    }

    // none found
    return {NAN, NAN};
}

bool DecisionMaking::isCloseEnough(geometry_msgs::msg::Pose a,
                    geometry_msgs::msg::PoseStamped b)
{
    // --- Position difference ---
    double dx = a.position.x - b.pose.position.x;
    double dy = a.position.y - b.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > 1.0)
        return false;

    // --- Quaternion → yaw conversion ---
    auto quatToYaw = [](const geometry_msgs::msg::Quaternion& q) {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    };

    double yawA = quatToYaw(a.orientation);
    double yawB = quatToYaw(b.pose.orientation);
    double yawDiff = std::fabs(yawA - yawB);

    // Normalize to [0, π]
    if (yawDiff > M_PI)
        yawDiff = 2 * M_PI - yawDiff;

    // --- 30 degrees = π/6 radians ---
    return yawDiff <= M_PI / 6;
}


void DecisionMaking::sendNextGoal(){
    if (!client_) {
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action client not initialised; cannot send goal");
        return;
    }

    NavigateToPose::Goal goal;
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        goal.pose = currentGoalPose_;
    }

    if (goal.pose.header.frame_id.empty()) {
        RCLCPP_WARN(this->get_logger(), "Next goal pose frame_id is empty; skipping send");
        return;
    }

    goal.pose.header.stamp = this->now();

    if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available; goal not sent");
        return;
    }

    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
        [this](std::shared_future<GoalHandleNavigateToPose::SharedPtr> future) {
            auto goal_handle = future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal rejected by action server");
            } else {
                RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted by action server");
            }
        };

    options.feedback_callback =
        [this](GoalHandleNavigateToPose::SharedPtr /*unused*/,
               const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
            if (!feedback) {
                return;
            }
            RCLCPP_DEBUG(
                this->get_logger(),
                "NavigateToPose feedback: remaining distance %.2f",
                feedback->distance_remaining);
        };

    options.result_callback =
        [this](const GoalHandleNavigateToPose::WrappedResult &result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "NavigateToPose goal succeeded");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "NavigateToPose goal cancelled");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "NavigateToPose goal ended with unknown result code");
                    break;
            }
        };

    auto future_goal_handle = client_->async_send_goal(goal, options);
    if (!future_goal_handle.valid()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send NavigateToPose goal");
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Sent NavigateToPose goal to /mission/navigate_to_pose at (%.3f, %.3f)",
        goal.pose.pose.position.x,
        goal.pose.pose.position.y);
}

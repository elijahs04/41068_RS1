#include "pathPlanning.h"

#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/qos.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <chrono>

namespace
{
constexpr double kEpsilon = 1e-6;
}

PathPlanning::PathPlanning() : rclcpp::Node("planning_execution")
{
    RCLCPP_INFO(this->get_logger(), "Planning action client started");

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "/mission/navigate_to_pose");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry",
        rclcpp::SensorDataQoS(),
        std::bind(&PathPlanning::odomCallback, this, std::placeholders::_1));

    pointCloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/hotspots/points_cloud",
        rclcpp::SensorDataQoS(),
        std::bind(&PathPlanning::pointCloudCallback, this, std::placeholders::_1));

    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_WARN(this->get_logger(), "Action server not available within timeout");
    }
}

PathPlanning::~PathPlanning()
{
    if (runThread.joinable()) {
        runThread.join();
    }
}

void PathPlanning::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    std::vector<std::pair<float, float>> new_points;
    new_points.reserve(msg->width * msg->height);

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
                                                     iter_y(*msg, "y");
         iter_x != iter_x.end();
         ++iter_x, ++iter_y) {
        const float x = *iter_x;
        const float y = *iter_y;

        if (!std::isfinite(x) || !std::isfinite(y)) {
            continue;
        }

        new_points.emplace_back(x, y);
    }

    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        xyPoints_ = std::move(new_points);
        updatePlanningStateLocked();
    }

    sendNextGoal();
}

void PathPlanning::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        currentPose_ = msg->pose.pose;
        updateClusterCoverageLocked();
        refreshActiveClusterLocked();
    }

    sendNextGoal();
}

void PathPlanning::updatePlanningStateLocked()
{
    rebuildClustersLocked();
    updateClusterCoverageLocked();
    refreshActiveClusterLocked();
}

void PathPlanning::rebuildClustersLocked()
{
    const auto now = this->now();
    const float distance_threshold_sq =
        static_cast<float>(clusterDistanceThreshold_ * clusterDistanceThreshold_);

    const auto &points = xyPoints_;
    if (points.empty()) {
        clusters_.clear();
        return;
    }

    std::vector<int> labels(points.size(), -1);
    std::vector<std::vector<size_t>> groups;
    groups.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        if (labels[i] != -1) {
            continue;
        }

        std::queue<size_t> queue;
        queue.push(i);

        labels[i] = static_cast<int>(groups.size());
        groups.emplace_back();

        while (!queue.empty()) {
            const size_t index = queue.front();
            queue.pop();
            groups.back().push_back(index);

            for (size_t j = 0; j < points.size(); ++j) {
                if (labels[j] != -1) {
                    continue;
                }

                const float dx = points[j].first - points[index].first;
                const float dy = points[j].second - points[index].second;
                if (dx * dx + dy * dy <= distance_threshold_sq) {
                    labels[j] = labels[i];
                    queue.push(j);
                }
            }
        }
    }

    std::vector<Cluster> updated_clusters;
    updated_clusters.reserve(groups.size());
    std::vector<bool> matched_existing(clusters_.size(), false);

    for (const auto &indices : groups) {
        if (indices.size() < minClusterSize_) {
            continue;
        }

        double sum_x = 0.0;
        double sum_y = 0.0;
        std::vector<std::pair<float, float>> cluster_points;
        cluster_points.reserve(indices.size());

        for (const auto idx : indices) {
            sum_x += static_cast<double>(points[idx].first);
            sum_y += static_cast<double>(points[idx].second);
            cluster_points.push_back(points[idx]);
        }

        const double centroid_x = sum_x / static_cast<double>(indices.size());
        const double centroid_y = sum_y / static_cast<double>(indices.size());

        double radius = 0.0;
        for (const auto &point : cluster_points) {
            const double dx = static_cast<double>(point.first) - centroid_x;
            const double dy = static_cast<double>(point.second) - centroid_y;
            radius = std::max(radius, std::hypot(dx, dy));
        }

        int best_index = -1;
        double best_distance = clusterMergeThreshold_;
        for (size_t existing_index = 0; existing_index < clusters_.size(); ++existing_index) {
            if (matched_existing[existing_index]) {
                continue;
            }

            const auto &existing = clusters_[existing_index];
            const double dx = existing.centroid.x - centroid_x;
            const double dy = existing.centroid.y - centroid_y;
            const double distance = std::hypot(dx, dy);
            if (distance <= best_distance) {
                best_distance = distance;
                best_index = static_cast<int>(existing_index);
            }
        }

        if (best_index >= 0) {
            Cluster cluster = clusters_[best_index];
            cluster.points = std::move(cluster_points);
            cluster.centroid.x = centroid_x;
            cluster.centroid.y = centroid_y;
            cluster.centroid.z = 0.0;
            cluster.radius = radius;
            cluster.last_seen = now;
            cluster.explored = !clusterHasUnvisited(cluster);
            updated_clusters.push_back(std::move(cluster));
            matched_existing[static_cast<size_t>(best_index)] = true;
        } else {
            Cluster cluster;
            cluster.id = nextClusterId_++;
            cluster.points = std::move(cluster_points);
            cluster.centroid.x = centroid_x;
            cluster.centroid.y = centroid_y;
            cluster.centroid.z = 0.0;
            cluster.radius = radius;
            cluster.last_seen = now;
            cluster.explored = false;
            updated_clusters.push_back(std::move(cluster));
        }
    }

    for (size_t index = 0; index < clusters_.size(); ++index) {
        if (!matched_existing[index] && !clusters_[index].explored) {
            updated_clusters.push_back(clusters_[index]);
        }
    }

    clusters_.swap(updated_clusters);

    if (pendingTargetCell_) {
        bool cell_still_present = false;
        for (const auto &cluster : clusters_) {
            for (const auto &point : cluster.points) {
                if (cellKeyForPoint(point.first, point.second) == *pendingTargetCell_) {
                    cell_still_present = true;
                    break;
                }
            }
            if (cell_still_present) {
                break;
            }
        }

        if (!cell_still_present) {
            pendingTargetCell_.reset();
            goalDispatched_ = false;
        }
    }
}

void PathPlanning::updateClusterCoverageLocked()
{
    if (clusters_.empty()) {
        return;
    }

    const double visit_radius_sq = visitRadius_ * visitRadius_;
    const double robot_x = currentPose_.position.x;
    const double robot_y = currentPose_.position.y;

    for (auto &cluster : clusters_) {
        for (const auto &point : cluster.points) {
            const double dx = static_cast<double>(point.first) - robot_x;
            const double dy = static_cast<double>(point.second) - robot_y;
            const double distance_sq = dx * dx + dy * dy;
            if (distance_sq > visit_radius_sq) {
                continue;
            }

            const int64_t key = cellKeyForPoint(point.first, point.second);
            cluster.visited_cells.insert(key);

            if (pendingTargetCell_ && key == *pendingTargetCell_) {
                pendingTargetCell_.reset();
                goalDispatched_ = false;
            }
        }

        cluster.explored = !clusterHasUnvisited(cluster);
    }
}

bool PathPlanning::refreshActiveClusterLocked()
{
    if (activeClusterId_) {
        const auto it = std::find_if(
            clusters_.begin(),
            clusters_.end(),
            [this](const Cluster &cluster) { return cluster.id == *activeClusterId_; });

        if (it == clusters_.end() || it->explored) {
            activeClusterId_.reset();
            pendingTargetCell_.reset();
            goalDispatched_ = false;
        }
    }

    if (!activeClusterId_) {
        const double robot_x = currentPose_.position.x;
        const double robot_y = currentPose_.position.y;

        double best_distance = std::numeric_limits<double>::infinity();
        std::optional<size_t> best_id;

        for (const auto &cluster : clusters_) {
            if (cluster.explored || !clusterHasUnvisited(cluster)) {
                continue;
            }

            const double dx = cluster.centroid.x - robot_x;
            const double dy = cluster.centroid.y - robot_y;
            const double distance = std::hypot(dx, dy);
            if (distance < best_distance) {
                best_distance = distance;
                best_id = cluster.id;
            }
        }

        if (best_id) {
            activeClusterId_ = best_id;
            pendingTargetCell_.reset();
            goalDispatched_ = false;
        }
    }

    if (!activeClusterId_) {
        return false;
    }

    return computeNextGoalForActiveClusterLocked();
}

bool PathPlanning::computeNextGoalForActiveClusterLocked()
{
    if (!activeClusterId_) {
        return false;
    }

    auto cluster_it = std::find_if(
        clusters_.begin(),
        clusters_.end(),
        [this](const Cluster &cluster) { return cluster.id == *activeClusterId_; });

    if (cluster_it == clusters_.end()) {
        activeClusterId_.reset();
        pendingTargetCell_.reset();
        goalDispatched_ = false;
        return false;
    }

    Cluster &cluster = *cluster_it;
    if (cluster.explored || !clusterHasUnvisited(cluster)) {
        cluster.explored = true;
        activeClusterId_.reset();
        pendingTargetCell_.reset();
        goalDispatched_ = false;
        return false;
    }

    if ((goalReady_ || goalDispatched_) && pendingTargetCell_) {
        return false;
    }

    const double robot_x = currentPose_.position.x;
    const double robot_y = currentPose_.position.y;

    double best_distance_sq = std::numeric_limits<double>::infinity();
    std::optional<std::pair<float, float>> target_point;
    int64_t target_cell = 0;

    for (const auto &point : cluster.points) {
        const int64_t key = cellKeyForPoint(point.first, point.second);
        if (cluster.visited_cells.count(key) > 0) {
            continue;
        }

        const double dx = static_cast<double>(point.first) - robot_x;
        const double dy = static_cast<double>(point.second) - robot_y;
        const double distance_sq = dx * dx + dy * dy;

        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            target_point = point;
            target_cell = key;
        }
    }

    if (!target_point) {
        cluster.explored = true;
        activeClusterId_.reset();
        pendingTargetCell_.reset();
        goalDispatched_ = false;
        return false;
    }

    const double distance = std::sqrt(best_distance_sq);
    if (distance <= kEpsilon) {
        RCLCPP_WARN(
            this->get_logger(),
            "Robot is coincident with target point for cluster %zu; skipping goal",
            cluster.id);
        cluster.visited_cells.insert(target_cell);
        cluster.explored = !clusterHasUnvisited(cluster);
        return false;
    }

    const double direction_x =
        (static_cast<double>(target_point->first) - robot_x) / distance;
    const double direction_y =
        (static_cast<double>(target_point->second) - robot_y) / distance;

    double standoff = goalStandOff_;
    if (distance < goalStandOff_) {
        standoff = std::max(0.5, distance * 0.5);
    }

    const double goal_x =
        static_cast<double>(target_point->first) - direction_x * standoff;
    const double goal_y =
        static_cast<double>(target_point->second) - direction_y * standoff;

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

    goalReady_ = true;
    pendingTargetCell_ = target_cell;

    RCLCPP_INFO(
        this->get_logger(),
        "Cluster %zu target (%.3f, %.3f); goal at (%.3f, %.3f) with %.2f m standoff",
        cluster.id,
        static_cast<double>(target_point->first),
        static_cast<double>(target_point->second),
        goal_x,
        goal_y,
        standoff);

    return true;
}

bool PathPlanning::clusterHasUnvisited(const Cluster &cluster) const
{
    for (const auto &point : cluster.points) {
        const int64_t key = cellKeyForPoint(point.first, point.second);
        if (cluster.visited_cells.count(key) == 0) {
            return true;
        }
    }
    return false;
}

int64_t PathPlanning::cellKeyForPoint(float x, float y) const
{
    const int32_t cell_x = static_cast<int32_t>(std::floor(x / cellSize_));
    const int32_t cell_y = static_cast<int32_t>(std::floor(y / cellSize_));
    return (static_cast<int64_t>(cell_x) << 32) ^
           (static_cast<int64_t>(cell_y) & 0xffffffff);
}

void PathPlanning::sendNextGoal()
{
    geometry_msgs::msg::PoseStamped goal;
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        if (!goalReady_ || goalDispatched_ || goalActive_) {
            return;
        }

        goal = currentGoalPose_;
        goalReady_ = false;
        goalDispatched_ = true;
    }

    dispatchGoal(goal);
}

void PathPlanning::dispatchGoal(const geometry_msgs::msg::PoseStamped &goal_pose)
{
    if (!client_->action_server_is_ready()) {
        RCLCPP_WARN(
            this->get_logger(),
            "NavigateToPose action server not ready; deferring goal dispatch");
        std::lock_guard<std::mutex> lock(dataMutex_);
        currentGoalPose_ = goal_pose;
        goalReady_ = true;
        goalDispatched_ = false;
        goalActive_ = false;
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Dispatching goal at (%.3f, %.3f) facing %.3f rad",
        goal_pose.pose.position.x,
        goal_pose.pose.position.y,
        std::atan2(
            2.0 * (goal_pose.pose.orientation.w * goal_pose.pose.orientation.z +
                   goal_pose.pose.orientation.x * goal_pose.pose.orientation.y),
            1.0 - 2.0 * (goal_pose.pose.orientation.y * goal_pose.pose.orientation.y +
                         goal_pose.pose.orientation.z * goal_pose.pose.orientation.z)));

    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
        std::bind(&PathPlanning::handleGoalResponse, this, std::placeholders::_1);
    options.feedback_callback =
        std::bind(&PathPlanning::handleFeedback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
        std::bind(&PathPlanning::handleResult, this, std::placeholders::_1);

    try {
        client_->async_send_goal(goal_msg, options);
        std::lock_guard<std::mutex> lock(dataMutex_);
        goalActive_ = true;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to send NavigateToPose goal: %s",
            ex.what());
        std::lock_guard<std::mutex> lock(dataMutex_);
        goalReady_ = true;
        goalDispatched_ = false;
        goalActive_ = false;
    }
}

void PathPlanning::handleGoalResponse(const GoalHandleNavigateToPose::SharedPtr &goal_handle)
{
    if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "NavigateToPose goal rejected by server");
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            goalActive_ = false;
            goalDispatched_ = false;
            goalReady_ = true;
        }
        sendNextGoal();
        return;
    }

    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        activeGoalHandle_ = goal_handle;
    }

    RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted");
}

void PathPlanning::handleFeedback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    if (!feedback) {
        return;
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "NavigateToPose feedback: distance remaining %.2f m",
        feedback->distance_remaining);
}

void PathPlanning::handleResult(const GoalHandleNavigateToPose::WrappedResult &result)
{
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        goalActive_ = false;
        goalDispatched_ = false;
        activeGoalHandle_.reset();
        pendingTargetCell_.reset();
        refreshActiveClusterLocked();
    }

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "NavigateToPose goal succeeded");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "NavigateToPose goal canceled");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal aborted");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal ended with unknown result code");
            break;
    }

    sendNextGoal();
}

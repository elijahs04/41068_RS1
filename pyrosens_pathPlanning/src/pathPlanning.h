#ifndef PathPlanning_H
#define PathPlanning_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <vector>
#include <utility>
#include <thread>
#include <mutex>
#include <optional>
#include <unordered_set>

class PathPlanning: public rclcpp::Node {
    public:

        using NavigateToPose = nav2_msgs::action::NavigateToPose;

        PathPlanning();
        ~PathPlanning();      


    private:

        double currentPathLength_;

        std::vector<std::pair<float, float>> xyPoints_;
        struct Cluster
        {
            size_t id;
            std::vector<std::pair<float, float>> points;
            std::unordered_set<int64_t> visited_cells;
            geometry_msgs::msg::Point centroid;
            double radius{0.0};
            rclcpp::Time last_seen;
            bool explored{false};
            size_t next_boundary_index{0};
        };
        std::vector<Cluster> clusters_;
        std::optional<size_t> activeClusterId_;
        size_t nextClusterId_{1};
        std::optional<int64_t> pendingTargetCell_;
        bool goalReady_{false};
        bool goalDispatched_{false};
        bool goalActive_{false};
        
        geometry_msgs::msg::Pose currentPose_;

        geometry_msgs::msg::PoseStamped currentGoalPose_; 

        std::mutex dataMutex_;

        

        std::thread runThread;
        
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
        GoalHandleNavigateToPose::SharedPtr activeGoalHandle_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalVizPub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plannedPathPub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr executedPathPub_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;


        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

        void sendNextGoal();
        void dispatchGoal(const geometry_msgs::msg::PoseStamped &goal_pose);
        void handleGoalResponse(const GoalHandleNavigateToPose::SharedPtr &future_handle);
        void handleFeedback(
            GoalHandleNavigateToPose::SharedPtr goal_handle,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void handleResult(const GoalHandleNavigateToPose::WrappedResult &result);

        void updatePlanningStateLocked();
        void rebuildClustersLocked();
        bool refreshActiveClusterLocked();
        bool computeNextGoalForActiveClusterLocked();
        void updateClusterCoverageLocked();
        void markPendingCellVisitedLocked();
        void appendExecutedPathLocked(const geometry_msgs::msg::Pose &pose);
        bool clusterHasUnvisited(const Cluster &cluster) const;
        std::vector<std::pair<float, float>> extractBoundaryPoints(
            const std::vector<std::pair<float, float>> &points,
            double centroid_x,
            double centroid_y) const;
        int64_t cellKeyForPoint(float x, float y) const;

        const double clusterDistanceThreshold_{1.0};
        const double clusterMergeThreshold_{1.5};
        const size_t minClusterSize_{3};
        const double visitRadius_{1.0};
        const double goalStandOff_{3.0};
        const double cellSize_{0.5};
        const size_t boundaryAngleBins_{72};
        const double minBoundaryRadius_{0.3};
        const double boundaryDistanceTolerance_{0.05};
        const size_t executedPathMaxSize_{2000};
        const double executedPathMinSpacing_{0.1};

        nav_msgs::msg::Path executedPath_;
        bool executedPathInitialized_{false};
        std::optional<nav_msgs::msg::Path> pendingPathMsg_;
};



#endif // PathPlanning_H

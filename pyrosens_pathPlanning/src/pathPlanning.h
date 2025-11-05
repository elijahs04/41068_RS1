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
        };
        std::vector<Cluster> clusters_;
        std::optional<size_t> activeClusterId_;
        size_t nextClusterId_{1};
        std::optional<int64_t> pendingTargetCell_;
        bool goalReady_{false};
        bool goalDispatched_{false};
        
        geometry_msgs::msg::Pose currentPose_;

        geometry_msgs::msg::PoseStamped currentGoalPose_; 

        std::mutex dataMutex_;

        

        std::thread runThread;
        
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;


        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

        void sendNextGoal();

        void updatePlanningStateLocked();
        void rebuildClustersLocked();
        bool refreshActiveClusterLocked();
        bool computeNextGoalForActiveClusterLocked();
        void updateClusterCoverageLocked();
        bool clusterHasUnvisited(const Cluster &cluster) const;
        int64_t cellKeyForPoint(float x, float y) const;

        const double clusterDistanceThreshold_{1.0};
        const double clusterMergeThreshold_{1.5};
        const size_t minClusterSize_{3};
        const double visitRadius_{1.0};
        const double goalStandOff_{3.0};
        const double cellSize_{0.5};
};



#endif // PathPlanning_H

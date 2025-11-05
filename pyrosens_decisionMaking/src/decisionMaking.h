#ifndef DECISIONMAKING_H
#define DECISIONMAKING_H

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
#include <thread>
#include <mutex>

class DecisionMaking: public rclcpp::Node {
    public:

        using NavigateToPose = nav2_msgs::action::NavigateToPose;

        DecisionMaking();
        ~DecisionMaking();      


    private:

        double currentPathLength_;

        std::vector<std::pair<float, float>> xyPoints_;
        
        geometry_msgs::msg::Pose currentPose_;

        geometry_msgs::msg::PoseStamped currentGoalPose_; 

        sensor_msgs::msg::PointCloud2::SharedPtr pointCloud_;
        std::mutex dataMutex_;

        

        std::thread runThread;
        
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;


        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
        
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

        void sendNextGoal();

        void findFirstGoal();
};



#endif // DECISIONMAKING_H
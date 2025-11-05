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
#include <thread>

class PathPlanning: public rclcpp::Node {
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        PathPlanning();
        ~PathPlanning();

        bool sendGoalAndWait(double x, double y, double qz, double qw, double theta);
        void runGoals();        


    private:

        double currentPathLength_;
        
        geometry_msgs::msg::Pose currentPose_;

        geometry_msgs::msg::PoseStamped currentGoalPose_; 

        std::thread runThread;
        
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
        
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void moveUpward(double meters);

        bool checkGoalReachable();

        void clearCostmap(); 
};



#endif // PathPlanning_H